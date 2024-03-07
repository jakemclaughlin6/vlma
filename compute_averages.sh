
run_average() 
{  
    ~/catkin_ws/devel/lib/vlma/vlma_average_roughness --pointcloud_path $1
}


test1='/home/jake/Downloads/CB1CB3_none.ply'
test2='/home/jake/Downloads/CB1CB3_geo.ply'
test3='/home/jake/Downloads/CB1CB3_sc.ply'
test4='/home/jake/Downloads/CB1CB3_scgeo.ply'
test5='/home/jake/Downloads/CB1CB2_none.ply'
test6='/home/jake/Downloads/CB1CB2_geo.ply'
test7='/home/jake/Downloads/CB1CB2_sc.ply'
test8='/home/jake/Downloads/CB1CB2_scgeo.ply'


# run_average $test1
# run_average $test2
# run_average $test3
# run_average $test4
# run_average $test5
# run_average $test6
run_average $test7
# run_average $test8
