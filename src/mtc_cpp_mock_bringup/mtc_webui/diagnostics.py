"""Read-only P2 contracts and a session event journal. Authored with OpenAI Codex.

Relationships are explicit producer declarations, never inferred from timing.
SQLite is the query index; events.jsonl remains the portable append-only record.
"""
from copy import deepcopy
import json
import re
import sqlite3
import uuid

from mtc_webui.observations import _base, _text, _check
from mtc_webui.mock_backend import ApiError

MODULES = ('task', 'pickplace', 'vision', 'calibration', 'planner', 'execution', 'tool', 'robot', 'system')


def goal_key(value):
    if not value:
        return None
    result = uuid.UUID(value).hex
    if result == '0' * 32:
        raise ValueError('Goal UUID 不能全为零')
    return result


def calibration_sample(data):
    _base(data, 'calibration')
    result = {key: _text(data, key, key == 'calibration_id') for key in ('calibration_id', 'evidence', 'reference', 'reason')}
    result['valid'] = _check(data, 'valid')
    if result['valid'] is True and (result['evidence'] != 'measured' or not result['reference']):
        raise ValueError('有效标定需要 measured 证据与可追溯 reference；名义/Fake 参数不算实测')
    return result


def trace_sample(data):
    _base(data, 'trace')
    result = {key: _text(data, key, key in ('link_id', 'task_id')) for key in ('link_id', 'task_id', 'command_id')}
    for key in ('execute_goal_id', 'pick_goal_id', 'fjt_goal_id'):
        result[key] = goal_key(data.get(key))
    if not result['execute_goal_id'] or not result['pick_goal_id']:
        raise ValueError('trace 必须显式提供 ExecuteTask 与 PickPlace UUID')
    ids = [result[k] for k in ('execute_goal_id', 'pick_goal_id', 'fjt_goal_id') if result[k]]
    if len(set(ids)) != len(ids):
        raise ValueError('不同 Action 不得使用同一 UUID')
    return result


def trace_log(message):
    """Exact existing C++ MTC_TRACE forms; no substring UUID guessing."""
    match = re.fullmatch(r'MTC_TRACE event=task_start parent=([0-9a-fA-F]{32}) task=("(?:[^"\\]|\\.)*")', message)
    if match:
        return dict(task_id=json.loads(match[2]), execute_goal_id=goal_key(match[1]))
    match = re.fullmatch(r'MTC_TRACE event=child_response parent=([0-9a-fA-F]{32}) child=([0-9a-fA-F]{32}) accepted=1 request=("(?:[^"\\]|\\.)*")', message)
    if match:
        parent, child = goal_key(match[1]), goal_key(match[2])
        if parent != child:
            return dict(execute_goal_id=parent, pick_goal_id=child, request_id=json.loads(match[3]))
    return None


class Journal:
    def __init__(self, output_dir=None):
        self.db = sqlite3.connect(str(output_dir / 'events.sqlite3') if output_dir else ':memory:', check_same_thread=False)
        self.db.execute('CREATE TABLE events (seq INTEGER PRIMARY KEY, task TEXT, goal TEXT, module TEXT, level TEXT, transition INTEGER, data TEXT)')
        self.db.execute('CREATE INDEX events_task ON events(task, seq)')
        self.db.execute('CREATE INDEX events_goal ON events(goal, seq)')
        self.nodes, self.edges = {}, {}
        self.persistent = output_dir is not None

    def append(self, event):
        self.db.execute('INSERT INTO events VALUES (?, ?, ?, ?, ?, ?, ?)',
                        (event['seq'], event.get('task_id'), goal_key(event.get('goal_id')), event['module'], event['level'],
                         bool(event.get('transition')), json.dumps(event, ensure_ascii=False, allow_nan=False)))
        self.db.commit()

    def node(self, goal, action, task_id=None):
        goal = goal_key(goal)
        node = self.nodes.setdefault(goal, dict(goal_id=goal, actions=set(), task_ids=set()))
        node['actions'].add(action)
        if task_id:
            node['task_ids'].add(task_id)
        return goal

    def declare(self, data, source, seq):
        parent = self.node(data['execute_goal_id'], 'ExecuteTask', data.get('task_id'))
        child = data.get('pick_goal_id')
        if child:
            child = self.node(child, 'PickPlace')
            self.edges.setdefault(child, {})[parent] = dict(parent=parent, child=child, source=source, seq=seq,
                                                         request_id=data.get('request_id'), command_id=data.get('command_id'))
        if data.get('fjt_goal_id'):
            fjt = self.node(data['fjt_goal_id'], 'FollowJointTrajectory')
            self.edges.setdefault(fjt, {})[child] = dict(parent=child, child=fjt, source=source, seq=seq,
                                                      command_id=data.get('command_id'))

    def _conflicts(self):
        bad = {g for g, n in self.nodes.items() if len(n['actions']) > 1 or len(n['task_ids']) > 1}
        bad.update(g for g, parents in self.edges.items() if len(parents) != 1)
        # A conflict invalidates descendants too; no uncertain edge can confer ownership.
        while True:
            more = {g for g, p in self.edges.items() if any(parent in bad for parent in p)} - bad
            if not more:
                return bad
            bad.update(more)

    def related(self, task_id=None, goal_id=None):
        bad = self._conflicts()
        selected = {goal_key(goal_id)} if goal_id else {g for g, n in self.nodes.items() if task_id in n['task_ids']} if task_id else set(self.nodes)
        while True:
            more = {g for g, p in self.edges.items() if g not in bad and any(parent in selected and parent not in bad for parent in p)} - selected
            if not more:
                return selected
            selected.update(more)

    def task_ids_for(self, goal):
        if not goal: return []
        goal = goal_key(goal)
        bad = self._conflicts()
        if goal in bad: return []
        result, seen = set(), set()
        while goal and goal not in seen:
            seen.add(goal)
            result.update(self.nodes.get(goal, {}).get('task_ids', set()))
            parents = self.edges.get(goal, {})
            goal = next(iter(parents)) if len(parents) == 1 else None
        return sorted(result)

    def graph(self, task_id=None, goal_id=None):
        chosen = self.related(task_id, goal_id)
        # Include every ancestor, even when the user searches by the FJT UUID.
        while True:
            parents = {p for child in chosen for p in self.edges.get(child, {})} - chosen
            if not parents: break
            chosen.update(parents)
        # Include declarations touching the selection to make conflicts reviewable.
        bad = self._conflicts()
        edges = [dict(e, conflict=child in bad) for child, parents in self.edges.items()
                 for e in parents.values() if child in chosen or e['parent'] in chosen]
        for edge in edges:
            chosen.update((edge['parent'], edge['child']))
        nodes = [dict(goal_id=g, action='/'.join(sorted(self.nodes[g]['actions'])), task_ids=sorted(self.nodes[g]['task_ids']),
                      conflict=g in bad) for g in sorted(chosen) if g in self.nodes]
        return dict(nodes=nodes, edges=edges, note='只显示显式 UUID 声明；缺失的层级保持未关联。日志声明不等于执行端停稳证据。')

    def query(self, *, task_id=None, goal_id=None, module=None, before=None, limit=100, transitions=False, errors=False, all_rows=False):
        if task_id and (not isinstance(task_id, str) or len(task_id) > 2000):
            raise ApiError(400, 'INVALID_QUERY', 'task_id 无效')
        try:
            goal_id = goal_key(goal_id)
        except (ValueError, AttributeError, TypeError):
            raise ApiError(400, 'INVALID_QUERY', 'Goal UUID 无效') from None
        if module and module not in MODULES:
            raise ApiError(400, 'INVALID_QUERY', '模块无效')
        if type(limit) is not int or not 1 <= limit <= 200 or (before is not None and (type(before) is not int or before <= 0)):
            raise ApiError(400, 'INVALID_QUERY', '分页参数无效')
        clauses, values = [], []
        if task_id:
            ids = sorted(self.related(task_id=task_id))
            clauses.append('(task = ? OR goal IN (' + ','.join('?' for _ in ids) + '))')
            values.extend([task_id] + ids)
        if goal_id:
            ids = sorted(self.related(goal_id=goal_id))
            clauses.append('goal IN (' + ','.join('?' for _ in ids) + ')')
            values.extend(ids)
        if module:
            clauses.append('module = ?'); values.append(module)
        if transitions:
            clauses.append('transition = 1')
        if errors:
            clauses.append("level IN ('warning','error')")
        where = ' WHERE ' + ' AND '.join(clauses) if clauses else ''
        total = self.db.execute('SELECT COUNT(*) FROM events' + where, values).fetchone()[0]
        failure_where = where + (' AND ' if clauses else ' WHERE ') + "level = 'error'"
        failure = self.db.execute('SELECT data FROM events' + failure_where + ' ORDER BY seq DESC LIMIT 1', values).fetchone()
        if before is not None:
            where += (' AND ' if clauses else ' WHERE ') + 'seq < ?'; values.append(before)
        sql = 'SELECT data FROM events' + where + ' ORDER BY seq DESC'
        if not all_rows:
            sql += ' LIMIT ?'; values.append(limit + 1)
        rows = [json.loads(r[0]) for r in self.db.execute(sql, values)]
        for event in rows:
            event['related_task_ids'] = self.task_ids_for(event.get('goal_id'))
        more = not all_rows and len(rows) > limit
        rows = rows[:limit] if not all_rows else rows
        return dict(events=rows, total=total, next_before=rows[-1]['seq'] if more else None,
                    latest_failure=json.loads(failure[0]) if failure else None, trace=self.graph(task_id, goal_id),
                    coverage=dict(persistent=self.persistent, complete_recorded_events=True,
                                  scope='本会话启动后的已接收事件；不含启动前、ROS 丢包或未提供关联的日志，不是完整 ROS bag'))
