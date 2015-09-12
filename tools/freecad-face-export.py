sel = Gui.Selection.getSelectionEx()
face = sel[0].SubObjects[0]
import Drawing
open('/tmp/out.dxf', 'w').write(Drawing.projectToDXF(face, App.Vector(0, 0, 1)))
