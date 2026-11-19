"""STEP -> metre STL, preserving original CAD coordinate axes and origin.
Run with isolated Python environment containing cadquery-ocp.
"""
from pathlib import Path
import hashlib,json
from OCP.STEPControl import STEPControl_Reader
from OCP.IFSelect import IFSelect_RetDone
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
from OCP.BRepBndLib import BRepBndLib
from OCP.Bnd import Bnd_Box
from OCP.StlAPI import StlAPI_Writer
from OCP.gp import gp_Trsf,gp_Pnt
import sys
source=Path(sys.argv[1]); out=Path(sys.argv[2]); out.mkdir(parents=True,exist_ok=True)
records=[]
for p in sorted(source.glob('*.stp')):
    reader=STEPControl_Reader()
    if reader.ReadFile(str(p)) != IFSelect_RetDone:
        records.append({'source':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'error':'STEP parser rejected file; no mesh generated'})
        print('FAILED:',p.name,flush=True)
        continue
    assert reader.TransferRoots()>0,p
    shape=reader.OneShape()
    transform=gp_Trsf();transform.SetScale(gp_Pnt(0,0,0),0.001)
    shape=BRepBuilderAPI_Transform(shape,transform,True).Shape()
    bounds=Bnd_Box();BRepBndLib.AddOptimal_s(shape,bounds)
    lo,hi=bounds.CornerMin(),bounds.CornerMax()
    box=(lo.X(),lo.Y(),lo.Z(),hi.X(),hi.Y(),hi.Z());dims=[box[i+3]-box[i] for i in range(3)]
    mesher=BRepMesh_IncrementalMesh(shape,0.0001,False,0.25,True);mesher.Perform()
    assert mesher.IsDone()
    name=p.stem.replace(' ','_')+'.stl';writer=StlAPI_Writer();writer.ASCIIMode=False
    assert writer.Write(shape,str(out/name))
    record={'source':str(p),'sha256':hashlib.sha256(p.read_bytes()).hexdigest(),'mesh':name,'unit':'m','source_unit':'mm','bounds_m':box,'dimensions_m':dims,'mounting_orientation':'unconfirmed'}
    records.append(record);print(p.name, 'dimensions mm:',[round(d*1000,2) for d in dims],flush=True)
(out/'cad_manifest.json').write_text(json.dumps(records,indent=2,ensure_ascii=False))
