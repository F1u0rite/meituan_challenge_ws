"""OpenCascade STEP solid backend, isolated from the existing vision runtime."""
from pathlib import Path
import hashlib, re, shutil, tempfile, sys
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RUNTIME=ROOT/'tools'/'cad_runtime'

def require_ocp():
    if RUNTIME.exists() and str(RUNTIME) not in sys.path:
        sys.path.insert(0,str(RUNTIME))
    try:
        import OCP
    except ImportError as exc:
        raise RuntimeError('CAD_BACKEND_UNAVAILABLE: use the documented Python 3.10 CAD runtime') from exc

def load_step(path):
    require_ocp()
    from OCP.STEPControl import STEPControl_Reader
    from OCP.IFSelect import IFSelect_RetDone
    from OCP.Interface import Interface_Static
    path=Path(path)
    if not path.is_file(): raise FileNotFoundError(path)
    text=path.read_text(encoding='utf-8',errors='replace')
    if not text.startswith('ISO-10303-21;'): raise ValueError('Not a STEP Part 21 file')
    if not re.search(r'SI_UNIT\s*\(\s*\.MILLI\.\s*,\s*\.METRE\.\s*\)',text):
        raise ValueError('Unsupported or missing STEP length unit; expected explicit millimetre SI unit')
    Interface_Static.SetCVal_s('xstep.cascade.unit','MM')
    with tempfile.TemporaryDirectory() as td:
        tmp=Path(td)/'solid.step';shutil.copyfile(path,tmp)
        reader=STEPControl_Reader()
        if reader.ReadFile(str(tmp))!=IFSelect_RetDone: raise ValueError('STEP entity reader failed')
        if reader.TransferRoots()<1: raise ValueError('STEP has no transferable solid roots')
        shape=reader.OneShape()
    if shape.IsNull(): raise ValueError('Empty STEP shape')
    from OCP.BRepCheck import BRepCheck_Analyzer
    from OCP.TopAbs import TopAbs_SOLID
    solids=list(explore(shape,TopAbs_SOLID))
    if not solids or not BRepCheck_Analyzer(shape).IsValid():
        raise ValueError('STEP is not a valid solid B-rep')
    metadata={'source_path':str(path.resolve()),'sha256':hashlib.sha256(path.read_bytes()).hexdigest(),
              'source_unit':'mm','kernel_unit':'mm','solid_count':len(solids),
              'step_entity_count':len(re.findall(r'#\d+\s*=',text)),
              'bbox_mm':bounds(shape),'volume_mm3':volume(shape)}
    return shape,metadata

def explore(shape,kind):
    from OCP.TopExp import TopExp_Explorer
    ex=TopExp_Explorer(shape,kind)
    while ex.More():
        yield ex.Current();ex.Next()

def bounds(shape):
    from OCP.Bnd import Bnd_Box
    from OCP.BRepBndLib import BRepBndLib
    b=Bnd_Box();BRepBndLib.AddOptimal_s(shape,b,False,False)
    return list(b.Get())

def volume(shape):
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    p=GProp_GProps();BRepGProp.VolumeProperties_s(shape,p)
    return float(abs(p.Mass()))

def planar_faces(shape):
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Plane
    from OCP.GProp import GProp_GProps
    from OCP.BRepGProp import BRepGProp
    found=[]
    for index,raw in enumerate(explore(shape,TopAbs_FACE),1):
        face=TopoDS.Face_s(raw);surf=BRepAdaptor_Surface(face)
        if surf.GetType()!=GeomAbs_Plane:continue
        plane=surf.Plane();p=GProp_GProps();BRepGProp.SurfaceProperties_s(face,p)
        found.append({'face_index':index,'normal':list(plane.Axis().Direction().Coord()),
                      'location_mm':list(plane.Location().Coord()),'bbox_mm':bounds(face),
                      'area_mm2':float(p.Mass())})
    return found

def transform_shape(shape,T_mm):
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    T=np.asarray(T_mm,dtype=float)
    if T.shape!=(4,4) or not np.allclose(T[3],[0,0,0,1]) or not np.allclose(T[:3,:3].T@T[:3,:3],np.eye(3),atol=1e-7) or not np.isclose(np.linalg.det(T[:3,:3]),1):
        raise ValueError('Not a proper rigid transform')
    tr=gp_Trsf();tr.SetValues(*T[:3,:].ravel().tolist())
    return BRepBuilderAPI_Transform(shape,tr,True).Shape()

def clip_box(shape,low,high):
    from OCP.gp import gp_Pnt
    from OCP.BRepPrimAPI import BRepPrimAPI_MakeBox
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    box=BRepPrimAPI_MakeBox(gp_Pnt(*map(float,low)),gp_Pnt(*map(float,high))).Shape()
    common=BRepAlgoAPI_Common(shape,box);common.Build()
    if not common.IsDone():raise RuntimeError('Solid clipping failed')
    return common.Shape()

def section_holes(shape,axis,coordinate):
    from OCP.gp import gp_Pln,gp_Pnt,gp_Dir
    from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeFace
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    from OCP.TopAbs import TopAbs_FACE,TopAbs_WIRE
    from OCP.TopoDS import TopoDS
    from OCP.BRepTools import BRepTools
    from OCP.BRepGProp import BRepGProp
    from OCP.GProp import GProp_GProps
    origin=np.zeros(3);origin[axis]=coordinate
    normal=np.zeros(3);normal[axis]=1
    face=BRepBuilderAPI_MakeFace(gp_Pln(gp_Pnt(*origin),gp_Dir(*normal)),-500,500,-500,500).Face()
    common=BRepAlgoAPI_Common(shape,face);common.Build()
    if not common.IsDone():raise RuntimeError('Solid section failed')
    holes=[]
    for raw_face in explore(common.Shape(),TopAbs_FACE):
        f=TopoDS.Face_s(raw_face);outer=BRepTools.OuterWire_s(f)
        for raw_wire in explore(f,TopAbs_WIRE):
            w=TopoDS.Wire_s(raw_wire)
            if w.IsSame(outer):continue
            mf=BRepBuilderAPI_MakeFace(w,True)
            if not mf.IsDone():continue
            p=GProp_GProps();BRepGProp.SurfaceProperties_s(mf.Face(),p)
            holes.append({'bbox_mm':bounds(w),'area_mm2':abs(float(p.Mass())),
                          'centroid_mm':list(p.CentreOfMass().Coord())})
    return holes

def mesh(shape,deflection_mm=.4):
    from OCP.BRepMesh import BRepMesh_IncrementalMesh
    from OCP.BRep import BRep_Tool
    from OCP.TopLoc import TopLoc_Location
    from OCP.TopAbs import TopAbs_FACE
    from OCP.TopoDS import TopoDS
    BRepMesh_IncrementalMesh(shape,deflection_mm,False,.3,True).Perform()
    triangles=[]
    for raw in explore(shape,TopAbs_FACE):
        face=TopoDS.Face_s(raw);loc=TopLoc_Location()
        tri=BRep_Tool.Triangulation_s(face,loc)
        if tri is None:continue
        for i in range(1,tri.NbTriangles()+1):
            indices=tri.Triangle(i).Get()
            triangles.append([list(tri.Node(j).Transformed(loc.Transformation()).Coord()) for j in indices])
    return np.asarray(triangles,float)

def interference(a,b):
    from OCP.BRepExtrema import BRepExtrema_DistShapeShape
    from OCP.BRepAlgoAPI import BRepAlgoAPI_Common
    distance=BRepExtrema_DistShapeShape(a,b);distance.Perform()
    if not distance.IsDone():return {'status':'UNKNOWN','reason':'DISTANCE_FAILED'}
    gap=float(distance.Value())
    if gap>1e-5:return {'status':'NOMINAL_CLEARANCE','distance_mm':gap,'overlap_mm3':0.}
    common=BRepAlgoAPI_Common(a,b);common.Build()
    if not common.IsDone():return {'status':'UNKNOWN','reason':'BOOLEAN_FAILED'}
    overlap=volume(common.Shape()) if not common.Shape().IsNull() else 0.
    return {'status':'COLLISION','distance_mm':gap,'overlap_mm3':overlap,
            'contact_only':overlap<=1e-5}
