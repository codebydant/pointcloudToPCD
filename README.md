# pointcloudToPCD
C++ application to convert ply file, txt file or xyz file point cloud to PCD file

-------------------
## How to compile
1. Download or clone repo: https://github.com/danielTobon43/pointcloudToPCD.git 
2. Create a "build" folder
the structre project should be something like:

	- build/
	- src/
	- cloud_test/
	- README.md
	
3. Modify the CMakeLists.txt file in */src* with your own PCL build dir: e.g. **/opt/pcl-1.9.0/build** and save it.	
4. Compile with cmake <br/>
at the root folder:

	- cd build/  
	- cmake ../src/
  	- make
       
        	 
### Test
Go to *pointcloudToPCD-master/build/bin* folder and:

    ./pointcloudToPCD <ply file> -o <output dir>
    ./pointcloudToPCD <txt file> -o <output dir>
    ./pointcloudToPCD <xyz file> -o <output dir>
    
    
 ## Note:
 The *.txt* file structure should only numbers: e.g.
 
	0.794 9.978 12.770 247 231 218
	0.793 9.979 12.769 234 217 207
	0.793 9.979 12.768 238 224 213
	0.794 9.979 12.767 239 227 215
	0.795 9.978 12.772 230 221 206
	0.795 9.978 12.771 243 229 216
	
Where each column represent: *x y z r g b*
 
 
 	

