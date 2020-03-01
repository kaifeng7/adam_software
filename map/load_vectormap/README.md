#protobuf 编译
在pc和Arm板上不兼容,因此移植时要重新编译proto

#protobuf 命令
proto --cpp_out=(输出路径) (proto路径)
如:proto -cpp_out= proto/map.proto 

##4.8日使用新的地图 zhiquan_v2
将之前4号路分为4,12,13三条路,因为开发地图人员不愿重画地图,所以没有按顺序排列地图id,这样发现了之前的一个bug

之前road顺序与id顺序一致,因此直接push_back进入数组,索引号就是id号

现在进行修改,将id号对应road_id,再将索引号与id号对应,使其方便查找.

#4.8fixed


