//#include "path_solve.h"
//#include <stdlib.h>
//#include <stdio.h>
//#include <math.h>
//#include "stdint.h"
//#include "self_headfile.h"
//#define W 500
//#define H 400
//#define MaxVertexNum 21

//int points_amount;
//double min_dist = 150 * 12;
//int p1[25][2]={0};
//struct GNode
//{
//	int Nv;
//	int Ne;
//	double G[MaxVertexNum][MaxVertexNum];
//	double end_dist[MaxVertexNum];
//};
//typedef struct GNode *MGraph;
//MGraph CreateGraph(int VertexNum)
//{
//	MGraph Graph = (MGraph)malloc(sizeof(struct GNode));
//	Graph->Nv = VertexNum;
//	Graph->Ne = VertexNum * VertexNum;
//	return Graph;
//}
//void add_Ne(MGraph Graph)
//{

//	for (int i = 0; i < Graph->Nv; i++)
//	{
//		int x0, y0;
//		if (i == 0)
//			x0 = 0, y0 = 0;
//		else 
//			x0 = pos_XY[i - 1][0], y0 = pos_XY[i - 1][1];
//		Graph->end_dist[i] = (int)(sqrt((x0 - W) * (x0 - W) + (y0 - H) * (y0 - H)));
//		for (int j = 0; j < Graph->Nv; j++)
//		{
//			int x1, y1;
//			if (j == 0)
//				x1 = 0, y1 = 0;
//			else
//				x1 = pos_XY[j - 1][0], y1 = pos_XY[j - 1][1];
//			Graph->G[i][j] = (int)(sqrt(((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1)) * 1.0));
//		}
//	}
//}


//void show_Graph(MGraph Graph)
//{
//	for (int i = 0; i < Graph->Nv; i++)
//	{
//		for (int j = 0; j < Graph->Nv; j++){
//			//printf("%4d ", (int)(Graph->G[i][j]));
//		}
//			
//		//printf("\n");
//	}
//}

//void show_path(int *path)
//{

//	for (int i = 0; i < points_amount + 1; i++)
//	{
//		//printf("%d->",path[i]);
//	}
//	//printf("\n");
//}
//void copy_path(int *path, int *shortest_path, int points_amount)
//{
//	for (int i = 0; i < points_amount + 1; i++)
//	{
//		shortest_path[i] = path[i];
//	}
//}
//void dfs(int position, int *visited, int *path, int points_amount, MGraph Graph, int *dist, int distance, int *shortest_path)
//{
//	if (position == points_amount + 1)
//	{	
//		distance += Graph->end_dist[path[position-1]];//最后一个点到右顶点的距离
//		//show_path(path);
//		if (distance <= min_dist)
//		{
//			//printf("%d\n", path[position - 1]);
//			min_dist = distance;
//			copy_path(path, shortest_path, points_amount);
//		}
//		return;
//	}
//	else if (distance > min_dist)
//	{
//		return;
//	}
//	for (int i = 0; i < points_amount; i++)
//	{
//		if (visited[i] != 1)
//		{
//			visited[i] = 1;
//			path[position] = i + 1;
//			int d = distance + Graph->G[path[position - 1]][i + 1];//选择该点后的路程
//			dfs(position + 1, visited, path, points_amount, Graph, dist, d, shortest_path);
//			visited[i] = 0;
//		}
//	}
//}
//void add_array(int p[][2], int points_amount, int mode)
//{

////	for (int i = 0; i <points_amount; i++)
////	{
////		if (mode)
////		{
////			p[i][0] = rand() % W;
////			p[i][1] = rand() % H;
////		}
////		else
////		{
////			p[i][0] = i * 100+100;
////			p[i][1] = i * 100+100;
////		}
////	}
//	p[0][0]=100;
//	p[0][1]=100;
//	
//	p[1][0]=100;
//	p[1][1]=200;
//	
//	p[2][0]=100;
//	p[2][1]=300;
//	
//	p[3][0]=300;
//	p[3][1]=100;
//	
//	p[4][0]=250;
//	p[4][1]=300;
//	
//	p[5][0]=350;
//	p[5][1]=400;
//	
//	p[6][0]=400;
//	p[6][1]=250;
//	
//}
//void print_array(int *p)
//{
//	for (int i = 0; i < points_amount + 1; i++)
//	{
//		//printf("%d %d\n",p[2 * i], p[2 * i + 1]);
//	}
//}



//void digui(int *visited, int *path, MGraph Graph, int *dist,int *shortest_path)
//{
//	dfs(1, visited, path, points_amount, Graph, dist, 0, shortest_path);
//	show_path(shortest_path);
//	//printf("min dist is %f \n", min_dist);
//}
//void copy_array()
//{
//	for (int i = 0; i < points_amount; i++)
//	{
//		pos_XY[i][0] = p1[i][0];
//		pos_XY[i][1] = p1[i][1];
////		lcd_showuint16(1,i,pos_XY[i][0]);
////		lcd_showuint16(50,i,pos_XY[i][1]);
//	}
//}
//void change(int* shortest_path)
//{

//	for (int i = 0; i < points_amount; i++)
//	{
//		//int temp_x = p[shortest_path[i + 1]-1][0], temp_y = p[shortest_path[i + 1]-1][1];
//		//p[shortest_path[i + 1] - 1][0] = p[i][0];
//		//p[shortest_path[i + 1] - 1][1] = p[i][1];
//		//p[i][0] = temp_x;
//		//p[i][1] = temp_y;
//		//printf("%d %d\n", temp_x, temp_y);
//		p1[i][0] = pos_XY[shortest_path[i + 1] - 1][0];
//		p1[i][1] = pos_XY[shortest_path[i + 1] - 1][1];
//	}
//	copy_array();
//}

//void show_point(int p[][2])
//{
//	for (int i = 0; i < points_amount; i++)
//	{
//		lcd_showuint16(1,i,p[i][0]);
//		lcd_showuint16(50,i,p[i][1]);
//	}
//}
//void path_solve(){
//	min_dist = 1000 * 20;

//	//srand((int)time(0));
//	/*points_amount = rand() % (21-5)+5;*/
//	points_amount = pos_sum;
//	//add_array((int(*)[2])pos_XY, points_amount, 0);//加入坐标，改写该函数

//	MGraph Graph = CreateGraph(points_amount + 1);
//	add_Ne(Graph);
//	int *visited = NULL;
//	visited = (int*)calloc(points_amount, sizeof(int));
//	int *path = NULL;
//	path = (int*)calloc(points_amount + 1, sizeof(int));//临时路径
//	//show_Graph(Graph);
//	//print_array(p);
//	path[0] = 0;//初始为原点
//	int *dist = NULL;
//	dist = (int*)calloc(points_amount, sizeof(int));
//	int *shortest_path = NULL;
//	shortest_path = (int*)calloc(points_amount, sizeof(int));//最短路径
//	digui(visited, path, Graph, dist, shortest_path);
//	change( shortest_path);
//	//show_point((int(*)[2])pos_XY);
//	
//}