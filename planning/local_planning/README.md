#local_planning

##参考OpenPlanner 进行局部路径规划

##4.18 
pathDensity = 1m
最后两个waypoint间差值,pathDensity=0.2m
计算减速度,进行减速

##4.26

增加倒車規劃
當m_CurrentPose與waypoint夾角大於90度時,速度爲負,即倒車
倒車時速度定爲minSpeed
