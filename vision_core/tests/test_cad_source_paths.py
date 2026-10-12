"""Windows-authored CAD provenance must still hash-check against Linux assets."""
import hashlib
import pytest
from perception.realtime_rgbd import verify_cad_sources


@pytest.mark.parametrize('prefix',[chr(92).join(['E:','work','assets','cad','']),'/work/assets/cad/'])
def test_foreign_source_paths_resolve_to_local_verified_assets(tmp_path,prefix):
    geometry={}
    for name in ('battery','hook'):
        filename=name+'.step'
        content=('private test provenance '+name).encode()
        (tmp_path/filename).write_bytes(content)
        geometry[name]=dict(source=dict(source_path=prefix+filename,
            sha256=hashlib.sha256(content).hexdigest()))
    verify_cad_sources(geometry,tmp_path)
    (tmp_path/'battery.step').write_bytes(b'changed model')
    with pytest.raises(ValueError,match='CAD cache source mismatch'):
        verify_cad_sources(geometry,tmp_path)


def test_missing_model_cannot_be_substituted(tmp_path):
    geometry={name:dict(source=dict(source_path=name+'.step',sha256='0'*64))
              for name in ('battery','hook')}
    with pytest.raises(FileNotFoundError):verify_cad_sources(geometry,tmp_path)
