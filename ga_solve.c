#include "ga_solve.h"
#include "stdio.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "stdint.h"
#include "self_headfile.h"

#define W 700
#define H 500
#define CITY_NUM 20     //城市编号是0-20
#define POPSIZE 100      //种群数量
#define MAXVALUE 100000   //路径值
#define N 20000            //需要根据实际求得的路径值修正

int Hash[CITY_NUM + 1];

int p1[25][2] = { 0 };

typedef struct CityPosition
{
	int x;
	int y;
}CityPosition;

double CityDistance[21][21];
double end_dist[22];
typedef struct {
	int colony[POPSIZE][CITY_NUM + 1];  //城市种群 1个是出发点
	double fitness[POPSIZE];          // 路径适应值
	double Distance[POPSIZE];         //路径实际长度
	int BestRooting[CITY_NUM + 1];      //最优城市路径序列
	double BestFitness;               //最优路径适应值
	double BestValue;                 //最优路径长度
}TSP, * PTSP;
void add_array(CityPosition* CityPos, int points_amount, int mode)
{
	CityPos[0].x = 0;
	CityPos[0].y = 0;
	for (int i = 1; i <= points_amount; i++)
	{
//		if (mode == 1)
//		{
//			CityPos[i].x = rand() % W;
//			CityPos[i].y = rand() % H;
//		}
//		else if (mode == 0)
//		{
//			/*cin >> p[2 * i] >> p[2 * i + 1];*/
//			CityPos[i].x = i*100;
//			CityPos[i].y = i * 100;
//			pos_XY[i][0] = i * 100;
//			pos_XY[i][1] = i * 100;
//		}
		if(mode == 1)
		{
			CityPos[i].x = pos_XY[i-1][0];
			CityPos[i].y = pos_XY[i-1][1];
		}
	}
}
//void print_array(CityPosition* CityPos, int points_amount)
//{
//	for (int i = 0; i < points_amount + 1; i++)
//	{
//		cout << CityPos[i].x << " " << CityPos[i].y << "\n";
//	}
//}
void CalculatDist(CityPosition* CityPos, int points_amount)
{//计算每两个城市之间的距离
	int i, j;
	int temp1, temp2;
	for (i = 0; i < points_amount + 1; i++) 
	{	
		int d1 = CityPos[i].x - W;
		int d2 = CityPos[i].y - H;
		end_dist[i] = sqrt(d1 * d1 + d2 * d2);
		for (j = 0; j <= points_amount + 1; j++) {
			temp1 = CityPos[j].x - CityPos[i].x;
			temp2 = CityPos[j].y - CityPos[i].y;
			CityDistance[i][j] = sqrt(temp1 * temp1 + temp2 * temp2);
		}
	}
}
//void show_Graph(double CityDistance[][21], int points_amount)
//{
//	for (int i = 0; i < points_amount + 1; i++)
//	{
//		for (int j = 0; j < points_amount + 1; j++)
//			printf("%4d ", int(CityDistance[i][j]));
//		cout << endl;
//	}
//}


void copy(int a[], int b[])
{//复制
	int i = 0;
	for (i = 0; i < CITY_NUM + 1; i++)
	{
		a[i] = b[i];
	}
}

uint8_t check(TSP* city, int pop, int num, int k)
{//用来检查新生成的节点是否在当前群体中，0号节点是默认出发节点和终止节点
	int i;
	for (i = 0; i <= num; i++) {
		if (k == city->colony[pop][i])
			return 1;//新生成节点存在于已经生成的路径中
	}
	return 0;//新生成节点没有存在于已经生成的路径中
}

void InitColony(TSP* city, int points_amount)
{//初始化种群
	int i, j, r;
	for (i = 0; i < POPSIZE; i++) {
		city->colony[i][0] = 0;
		city->BestValue = MAXVALUE;
		city->BestFitness = 0;//适应值越大越好
	}
	for (i = 0; i < POPSIZE; i++)
	{
		for (j = 1; j < points_amount + 1; j++)
		{
			r = rand() % points_amount + 1;//产生1～points_amount之间的随机数
			while (check(city, i, j, r))
			{
				r = rand() % points_amount + 1;
			}
			city->colony[i][j] = r;
		}
	}
}
void show_city(TSP* city, int points_amount)
{
	for (int i = 0; i < POPSIZE; i++)
	{
		for (int j = 0; j < points_amount + 1; j++)
			printf("%2d ", city->colony[i][j]);
//		cout << endl;
	}

}
void CalFitness(TSP* city, int points_amount)
{//计算适应值
	int i, j, start, end;
	int Best = 0;
	for (i = 0; i < POPSIZE; i++) {//求适应值
		city->Distance[i] = 0;
		for (j = 1; j < points_amount + 1; j++)
		{
			start = city->colony[i][j - 1]; end = city->colony[i][j];
			city->Distance[i] = city->Distance[i] + CityDistance[start][end];
		}
		city->Distance[i] += end_dist[city->colony[i][points_amount]];
		city->fitness[i] = pow(2, N / (city->Distance[i]));
		if (city->fitness[i] > city->fitness[Best])
			Best = i;
	}
	copy(city->BestRooting, city->colony[Best]);
	city->BestFitness = city->fitness[Best];
	city->BestValue = city->Distance[Best];
}

void Select(TSP* city)
{//选择算子
	int TempColony[POPSIZE][CITY_NUM + 1];
	int i, j, s, t;
	double GaiLv[POPSIZE];
	int SelectP[POPSIZE + 1];
	double avg;
	double sum = 0;
	for (i = 0; i < POPSIZE; i++)
	{
		sum += city->fitness[i];
	}
	for (i = 0; i < POPSIZE; i++)
	{
		GaiLv[i] = city->fitness[i] / sum;
	}
	SelectP[0] = 0;
	for (i = 0; i < POPSIZE; i++)
	{
		SelectP[i + 1] = SelectP[i] + GaiLv[i] * RAND_MAX;//轮盘数组生成
	}
	for (t = 0; t < POPSIZE; t++)
	{//轮盘赌
		s = rand() % RAND_MAX;
		for (i = 1; i < POPSIZE; i++)
		{
			if (SelectP[i] >= s)
				break;
		}
		copy(TempColony[t], city->colony[i - 1]);
	}
	for (i = 0; i < POPSIZE; i++)
	{
		copy(city->colony[i], TempColony[i]);
	}
}
void zeros(int Hash[])
{
	for (int i = 0; i < CITY_NUM + 1; i++)
	{
		Hash[i] = 0;
	}
}
void Cross(TSP* city, double pc, int points_amount, int Cross_length)
{//交叉算子
	int i, j, t;
	int a, b;
	int Temp1[CITY_NUM + 1], Temp2[CITY_NUM + 1];
	for (i = 0; i < POPSIZE - 1; i++)
	{
		double s = ((double)(rand() % RAND_MAX)) / RAND_MAX;
		if (s < pc)
		{
			a = rand() % (points_amount + 1 - Cross_length) + 1;//基因提取点
			//memset(Hash, 0, sizeof(Hash));//内存空间初始化
			zeros(Hash);
			Temp1[0] = Temp1[CITY_NUM] = 0;
			for (j = 1; j < Cross_length + 1; j++)
			{
				Temp1[j] = city->colony[i + 1][a + j - 1];
				Hash[Temp1[j]] = 1;
			}
			for (t = 1; t < points_amount + 1; t++)
			{
				if (Hash[city->colony[i][t]] == 0)  //该点未被访问
				{
					Temp1[j++] = city->colony[i][t];
					Hash[city->colony[i][t]] = 1;
				}
			}
			//memset(Hash, 0, sizeof(Hash));
			zeros(Hash);
			Temp2[0] = Temp2[CITY_NUM] = 0;
			for (j = 1; j < Cross_length + 1; j++)
			{
				Temp2[j] = city->colony[i][a + j - 1];
				Hash[Temp2[j]] = 1;
			}
			for (t = 1; t < points_amount + 1; t++)
			{
				if (Hash[city->colony[i + 1][t]] == 0)
				{
					Temp2[j++] = city->colony[i + 1][t];
					Hash[city->colony[i + 1][t]] = 1;
				}
			}
			copy(city->colony[i], Temp1);
			copy(city->colony[i + 1], Temp2);
		}
	}
}

double GetDistance(int a[CITY_NUM + 1], int points_amount)
{
	int i, start, end;
	double Distance = 0;
	for (i = 0; i < points_amount; i++)
	{
		start = a[i];   end = a[i + 1];
		Distance += CityDistance[start][end];
	}
	Distance+=end_dist[a[points_amount]];
	return Distance;
}

void Mutation(TSP* city, double pm, int points_amount)
{//变异算子
	int i;
	int Temp[CITY_NUM + 1];
	for (i = 0; i < POPSIZE; i++)
	{
		double s = ((double)(rand() % RAND_MAX)) / RAND_MAX;
		if (s < pm)
		{
			int a, b, t;
			a = (rand() % (points_amount)) + 1;
			b = (rand() % (points_amount)) + 1;
			copy(Temp, city->colony[i]);
			t = Temp[a];
			Temp[a] = Temp[b];
			Temp[b] = t;
			if (GetDistance(Temp, points_amount) > GetDistance(city->colony[i], points_amount))//重变异
			{
				a = (rand() % (points_amount)) + 1;
				b = (rand() % (points_amount)) + 1;
				copy(Temp, city->colony[i]);
				t = Temp[a];
				Temp[a] = Temp[b];
				Temp[b] = t;
			}
			copy(city->colony[i], Temp);
		}
	}
}

void OutPut(TSP* city, int points_amount)
{
	int i, j;
	printf("The population is:\n");
	for (i = 0; i < POPSIZE; i++)
	{
		for (j = 0; j <= points_amount; j++)
		{
			printf("%5d", city->colony[i][j]);
		}
		printf("    %f\n", city->Distance[i]);
	}
	printf("The best rooting is:\n");
	for (i = 0; i <= points_amount; i++)
		printf("%5d", city->BestRooting[i]);
	printf("\nIt's cost value is %f\n", (city->BestValue));
}
void copy_array(int points_amount)
{
	for (int i = 0; i < points_amount; i++)
	{
		pos_XY[i][0] = p1[i][0];
		pos_XY[i][1] = p1[i][1];

	}
}
void change(TSP* city,int points_amount)
{
	for (int i = 0; i < points_amount; i++)
	{
		p1[i][0] = pos_XY[city->BestRooting[i + 1] - 1][0];
		p1[i][1] = pos_XY[city->BestRooting[i + 1] - 1][1];
	}
	copy_array(points_amount);
}

double mygenus(uint8_t points_amount)
{
	
	CityPosition* CityPos = (CityPosition*)malloc(sizeof(CityPosition) * (points_amount + 1));
	add_array(CityPos, points_amount, 1);
	//print_array(CityPos,points_amount);

	TSP city;
	double pcross = 0.5, pmutation = 0.2;//交叉概率和变异概率
	int MaxEpoc = 700;//最大迭代次数
	int i;
	srand((unsigned)get_time_ms());//随机数发生器的初始化
	CalculatDist(CityPos, points_amount);//求城市间两两之间的距离
	//show_Graph(CityDistance, points_amount);

	InitColony(&city, points_amount);//生成初始种群
	//show_city(city, points_amount);

	CalFitness(&city, points_amount);//计算适应值,考虑应该在这里面把最优选出来
	for (i = 0; i < MaxEpoc; i++)
	{
		Select(&city);//选择(复制)
		Cross(&city, pcross, points_amount, 3);//交叉
		Mutation(&city, pmutation, points_amount);//变异
		CalFitness(&city, points_amount);//计算适应值		
	}
	OutPut(&city, points_amount);//输出
	change(&city, points_amount);
	return city.BestValue;
}

	



