# pointcloudToPCD
C++ application to convert ply file, txt file or xyz file point cloud to PCD file

-------------------
## Compile
* Create a "build" folder

in the main folder:

	- cd build  
	- cmake ../src/
  	- make
       
        	 
### Test

    ./pointcloudToPCD <ply file> -o <output dir>
    ./pointcloudToPCD <txt file> -o <output dir>
    ./pointcloudToPCD <xyz file> -o <output dir>

