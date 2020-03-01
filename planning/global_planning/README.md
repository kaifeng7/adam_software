#全局路径规划

##算法
DFS+Dijkstra
以每条路径为一个顶点,两条路径相连就有了边,边长为下一条路的长度(如a路与b路相连,a,b为节点,a->b的边长为b路的全长)
按理说应该用junction为顶点,道路长度为边建图,但是已经写好了就不想改了= =,而且好像用junction为顶点也会遇到麻烦

比较麻烦的点在于不知道道路的正反连接关系,每次将道路相连都需要比较两条路首尾的距离(即比较四个点,距离最近的即为相连,有a头接b尾,b头接a尾,a头接a尾,b头接b尾等4种情况)

##遗留问题

当起点和终点在一条路上,而选择换路会更近时,全局路径规划还是选择较远的同一条路径

##解决方法
在算法上修改比较麻烦,解决方法是在地图端解决= =,重新画地图,使每条路长度区别不大,这样就不会出现问题(在实际生活中,也不会出现一条非常长的且多个转弯的道路)
