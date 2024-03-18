
run_average() 
{  
    ~/catkin_ws/devel/lib/vlma/vlma_average_roughness --pointcloud_path $1
}


# test1='/home/jake/Downloads/CB1CB3_none.ply'
# test2='/home/jake/Downloads/CB1CB3_geo.ply'
# test3='/home/jake/Downloads/CB1CB3_sc.ply'
# test4='/home/jake/Downloads/CB1CB3_scgeo.ply'
# test5='/home/jake/Downloads/CB1CB2_none.ply'
# test6='/home/jake/Downloads/CB1CB2_geo.ply'
# test7='/home/jake/Downloads/CB1CB2_sc.ply'
# test8='/home/jake/Downloads/CB1CB2_scgeo.ply'

# test1='/home/jake/Downloads/SL2SL1_none.ply'
# test2='/home/jake/Downloads/SL2SL1_geo.ply'
# test3='/home/jake/Downloads/SL2SL1_sc.ply'
# test4='/home/jake/Downloads/SL2SL1_scgeo.ply'
# test5='/home/jake/Downloads/SL2SL3_none.ply'
# test6='/home/jake/Downloads/SL2SL3_geo.ply'
# test7='/home/jake/Downloads/SL2SL3_sc.ply'
# test8='/home/jake/Downloads/SL2SL3_scgeo.ply'


# run_average $test1
# run_average $test2
# run_average $test3
# run_average $test4
# echo "SL2SL3_none"
# run_average $test5
# echo "SL2SL3_geo"
# run_average $test6
# echo "SL2SL3_sc"
# run_average $test7
# echo "SL2SL3_scgeo"
# run_average $test8

# echo "SL2SL3_vma"
# test10='/home/jake/Downloads/SL2SL3_vma.ply'
# run_average $test10

# echo "SL2SL3_lma"
# test11='/home/jake/Downloads/SL2SL3_lma.ply'
# run_average $test11

# echo "SL2SL1_vma"
# test12='/home/jake/Downloads/SL2SL1_vma.ply'
# run_average $test12

# echo "SL2SL1_lma"
# test13='/home/jake/Downloads/SL2SL1_lma.ply'
# run_average $test13



# echo "CB1CB2_vma"
# test14='/home/jake/Downloads/CB1CB2_vma.ply'
# run_average $test14

# echo "CB1CB2_lma"
# test15='/home/jake/Downloads/CB1CB2_lma.ply'
# run_average $test15

# echo "CB1CB3_vma"
# test16='/home/jake/Downloads/CB1CB3_vma.ply'
# run_average $test16

# echo "CB1CB3_lma"
# test17='/home/jake/Downloads/CB1CB3_lma.ply'
# run_average $test17


test1='/home/jake/Downloads/SL2SL1_rigid.ply'
test2='/home/jake/Downloads/SL2SL3_rigid.ply'
test3='/home/jake/Downloads/CB1CB2_rigid.ply'
test4='/home/jake/Downloads/CB1CB3_rigid.ply'
run_average $test1
run_average $test2
run_average $test3
run_average $test4