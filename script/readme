The process flow is pretty simple for these. 

On every mesh, run ./runEachTime.sh in{MeshNum}.ply filtered{MeshNum}.ply
For the first two point clouds, run ./runEachPair.sh filtered1.ply filtered2.plyRegisteredTemp.ply

After that, run ./runEachPair.sh RegisteredTemp.ply filtered{MeshNum}.ply RegisteredTemp.ply

If you want more data for debugging, keep the temp files seperate rather than overwriting as you go along.

When everything is stitched together, run ./runFinally.sh RegisteredTemp.ply
