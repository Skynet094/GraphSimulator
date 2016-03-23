# include "iGraphics.h"
# include <string.h>
#include <limits.h>

int flag_bfs=0;
int flag_dfs=0;
int flag_mst=0;
int flag_dijkstra=0;
int simulation_image=1;

int stage_1=0;
int stage_2=0;
int stage_3=0;  //flow control variables 
int stage_4=0;
int stage_5=0;
int source_choose_stage=1;
int source_D,dest_D;
int weighted=0;
int flag_D=0;
//text field data start

int char_flag=0;     //text field variables
int char_top=-1;
#define char_MAX 30
int max_char=char_MAX;
int flag_off=1;
int flag_textfield=1;
int char_x=10;
unsigned long long  numgen(void);
void blink_control(void);
int image_flag=0;
unsigned long long int value=0;



struct  stack
{ 
 char num[2];
 double pos;
}stack[50];

void iText_Field(char* text)
{
	//TEXT FIELD PORTION START
	
	iSetcolor(0,0,64);
	iFilledRectangle(0,20,640,80);
	iSetcolor(255,255,255);
	iFilledRectangle(320,30,310,60);
	iText(20,40,text,GLUT_BITMAP_TIMES_ROMAN_24);
	iSetcolor(0,0,0);
	
	
	
	if(char_top>=0)
	for(int i=0;i<=char_top;i++)
		iText(stack[i].pos,50,stack[i].num,GLUT_BITMAP_TIMES_ROMAN_24);   //printing the numbers


		if(char_flag==0)
	{
		iLine(char_x+315,80,315+char_x,35);     //blinking portion
		char_flag=1;
		
	}

}
//text field data finish


//graph input data start

int cost;
int flag=0;
	//graph variables , vertex adding
#define MAX 100
int selection_flag1=-1;
int selection_flag2=-1;
int select_source=-1;

int count_edge=1;
int node_count=1;
#define MAX_NODE 100
unsigned long long int vertex_num,edge_num;
int adj_mat[MAX_NODE][MAX_NODE];
void adj_mat_print(void);
//graph input data finish


/// DFS data start
int dfs_stack[MAX_NODE];
int visited[MAX_NODE];
int dfs_top=-1;
int save_visited[MAX_NODE][MAX_NODE];
int iter_dfs=1;
int dfs_count=1;
int dfs_path[MAX_NODE][MAX_NODE];

struct dfs_track

{
    int node;
    int prev_node;

}dfs_track[MAX_NODE];
	
void pop(void)

{
if(dfs_top>=0)
{  int top_val=dfs_stack[dfs_top--];
    return;
}
}

void push(int n)
{
    dfs_stack[++dfs_top]=n;
}

void print_stack(void)
{
printf("\n\n");
for(int i=dfs_top;i>=0;i--)
    printf("%d\n",dfs_stack[i]);
printf("\n\n");
}



void dfs(int source_node)
{

visited[source_node]=-1;
save_visited[iter_dfs][1]=source_node;

push(source_node);
print_stack();
while(dfs_top!=-1)
{
        
       
		for(int i=1;i<=vertex_num;i++)
            for(int j=1;j<=vertex_num;j++)
                {

                  if(adj_mat[source_node][j]!=0 && visited[j]!=-1)
                  {
                      push(j);
                    visited[j]=-1;
                    dfs_track[++iter_dfs].node=j;
                    dfs_track[iter_dfs].prev_node=source_node;
                    save_visited[iter_dfs][1]=j;
                      print_stack();
                    source_node=j;
                    
                    break;
                  }
                }
		int flag_b=0;
		for(int i=1;i<vertex_num;i++)
			if(adj_mat[source_node][i])
			{
				if(visited[i]!=-1)
				{flag_b=1;
				break;
				}
			}

			if(!flag_b)
				pop();
			source_node=dfs_stack[dfs_top];

			     }


printf("\n\n");
for(int i=1;i<=iter_dfs;i++)
    for(int j=1;j<=i;j++)
        dfs_path[i][j]=save_visited[j][1];



for(int i=1;i<=iter_dfs;i++)
{for(int j=1;j<=vertex_num;j++)
    printf("%d\t",dfs_path[i][j]);
    printf("\n");
}

}


void dfs_count_control(void)

{
	if((dfs_count>=1 && dfs_count<=iter_dfs+1) && stage_5==1)
		dfs_count++;
}
//dfs data finish





//bfs data start

int visited_bfs[MAX_NODE];
int queue[MAX_NODE];
int queue_front=0;
int queue_count=0;
int iter_bfs=1;
int visit_index=0;
int save_visited_bfs[MAX_NODE][MAX_NODE];
int save_source_bfs[MAX_NODE][MAX_NODE];
int iter_pos=1;
int black_mat[MAX_NODE][MAX_NODE];
int grey_mat[MAX_NODE][MAX_NODE];
int count_bfs=1;

int dequeue(void)
{
    if(queue_count!=0)
{
    int front_val=queue[queue_front];
    queue_front = (queue_front+1) % vertex_num;
    queue_count--;
    return front_val;
}
}

void enqueue(int n)

{
if(( queue_count-queue_front)!=vertex_num)
    {
    queue[((queue_count+queue_front) % vertex_num)]=n;
    queue_count++;
    }
}

void print_queue(void)

{
    printf("\n\n\n");
for(int i=0;i<queue_count;i++)
    printf("%d\n",queue[(queue_front+i)% vertex_num]);
printf("\n\n\n\n");
}



void bfs(int source_node_bfs)

{
enqueue(source_node_bfs);

while((queue_count)!=0)
{
    visit_index=1;
    source_node_bfs=dequeue();
    save_source_bfs[iter_bfs][1]=source_node_bfs;

    visited_bfs[source_node_bfs]=-1;

    for(int i=1;i<=vertex_num;i++)
        {
        if(adj_mat[source_node_bfs][i]!=0 && visited_bfs[i]!=-1)
            {
            enqueue(i);
            save_visited_bfs[iter_bfs][visit_index++]=i;
            visited_bfs[i]=-1;
            }
        }
    print_queue();
    iter_bfs++;
}

for(int i=1;i<=iter_bfs;i++)
    for(int j=1;j<=i;j++)
                black_mat[i][j]=save_source_bfs[j][1];


printf("\n\n\n Source save:\n\n\n");

for(int i=1;i<=vertex_num;i++)
        {for(int j=1;j<=vertex_num;j++)
                printf("%d\t",black_mat[i][j]);
                printf("\n");
        }

    printf("\n\n\n");
    printf("Visit Save:\n\n\n");
for(int i=1;i<=vertex_num;i++)

{for(int j=1;j<=vertex_num;j++)
                printf("%d\t",save_visited_bfs[i][j]);
                printf("\n");

}

}



void bfs_count_control(void)

{
	if((count_bfs>=1 && count_bfs<=iter_bfs+1) && stage_5==1)
		count_bfs++;
}


//bfs data finish


//dijkstra data start

#define INFINITY 1<<16
#define BLACK 1  //flags
#define GREY 2

int iter_dijkstra=1;
int count_dijkstra=1;
int dijkstra_over=0;
int iter_track=1;

struct node_cost

{
    int cost;
    int color;

} node_cost[MAX_NODE][MAX_NODE];

struct Dijsktra_path_track

{
    int node;
    int prev_node;
}track_path_D[MAX_NODE];



int shortest_path[MAX_NODE];  //for taking Graph as input
int path_length=0;
int path_mat[MAX_NODE];

typedef struct VERTEX         //vertex properties required for Dijkstra's algorithm
{
int previous_node;
int color;
unsigned long long int distance;
}VERTEX;


int Dijkstra(int source_node , int dest_node )

{

int nodes=1 , i , temp_path[MAX_NODE];
VERTEX path_info[MAX_NODE];


for(int i=1;i<=vertex_num;i++)  //initializing the structure pat_info   //sob gula re grey kore dilam , r distance infinity kore dilam
    {
    path_info[i].distance=INFINITY;
     path_info[i].color=GREY;
     node_cost[iter_dijkstra][i].color=GREY;
     node_cost[iter_dijkstra][i].cost=-1;
     }


        path_info[source_node].color=BLACK;    //source node blacked out kore , er distance zero kore dilum
        path_info[source_node].distance=0;
        path_info[source_node].previous_node=0;

        node_cost[iter_dijkstra][source_node].color=BLACK;
        node_cost[iter_dijkstra][source_node].cost=0;
        iter_dijkstra++;

    do
    {



for(int i=1;i<=vertex_num;i++)  //initializing the structure pat_info   //sob gula re grey kore dilam , r distance infinity kore dilam
    {
     node_cost[iter_dijkstra][i].color=node_cost[iter_dijkstra-1][i].color;
     node_cost[iter_dijkstra][i].cost=node_cost[iter_dijkstra-1][i].cost;
     }

    for(i=1;i<=vertex_num;i++)

        {      if(adj_mat[source_node][i]>0  && path_info[i].color==GREY)

        {


                        if((path_info[source_node].distance+adj_mat[source_node][i])< path_info[i].distance)
        {
                                    path_info[i].distance=path_info[source_node].distance+adj_mat[source_node][i];
                                    path_info[i].previous_node=source_node;
                                    node_cost[iter_dijkstra][i].cost=path_info[i].distance;

        }
        }
}

int min=INFINITY,i;
source_node=0;

for(i=1;i<vertex_num+1;i++)
        if((path_info[i].color==GREY) && (path_info[i].distance<min))
                {source_node=i;
                min=path_info[i].distance;
                }

    if(source_node==0)       //shesh iteration e source_node==0  hobena (mone hote pare je , shesh iteration e sob gulo black tai source_node er value 0 thake , but asole sob gula black hobena):D cause shesh iteration e , source_node==dest_node hobe
        return 0;

    path_info[source_node].color=BLACK;
    node_cost[iter_dijkstra][source_node].color=BLACK;
    iter_dijkstra++;
}while(source_node!=dest_node);

/** path_mat genarate kore , minimum path_distance ber kora and last e node_num pass kora nicher portion er intended kaj**/

int temp_mat[MAX_NODE];
source_node=dest_node;
int iter_path=vertex_num;
do
{       temp_mat[nodes]=source_node;
		track_path_D[iter_path].node=source_node;
        track_path_D[iter_path--].prev_node=path_info[source_node].previous_node;
         source_node=path_info[source_node].previous_node;
         nodes++;

}while(source_node!=0);


for(i=1;i<nodes;i++)
    path_mat[i]=temp_mat[nodes-i];

for(int i=1;i<vertex_num;i++)
    path_length+=adj_mat[path_mat[i]][path_mat[i+1]];
 
printf("Minimum Path:\n");
    for(int i=1;i<nodes;i++)
        printf("%d\t",path_mat[i]);        //printing path_mat
        printf("\n\n");

    printf("Minimum Length = %d\n",path_length);


    for(int i=1;i<=vertex_num;i++)   //the simulation structure
    {for(int j=1;j<=vertex_num;j++)

        printf("%d\|%d   ",node_cost[i][j].color,node_cost[i][j].cost);
        printf("\n\n");
    }

return nodes;
}


void dijkstra_count_control(void)

{
	if((count_dijkstra>=1 && count_dijkstra<=iter_dijkstra) && stage_5==1)
		count_dijkstra++;
}


void dijkstra_track_control(void)

{
	if((iter_track>=1 && iter_track<=vertex_num)  && dijkstra_over==1)
		iter_track++;
}


//dijkstra data finish


//minimum spanning  tree data start
int mst_complete=0;
int iter_mst=1;
int count_mst;

struct track_mst

{
    int node;
    int prev_node;
    
}track_mst[MAX_NODE];


int minKey(int key[], int mstSet[])
{

   int min = INT_MAX, min_index;

   for (int v = 1; v <= vertex_num; v++)
     if (mstSet[v] == 0 && key[v] < min)
         min = key[v], min_index = v;

   return min_index;
}


void printMST(int parent[], int n, int adj_mat[MAX_NODE][MAX_NODE])
{
   printf("Edge   Weight\n");
   for (int i = 2; i <=vertex_num; i++)
      {printf("%d - %d    %d \n", parent[i], i, adj_mat[i][parent[i]]);
   track_mst[i].node=parent[i];
   track_mst[i].prev_node=i;
   count_mst++;
   }
   mst_complete=1;

}

void primMST(void)
{




     int parent[MAX_NODE]; // Array to store constructed MST
     int key[MAX_NODE];   // Key values used to pick minimum weight edge in cut
     int mstSet[MAX_NODE];  // To represent set of vertices not yet included in MST


     for (int i = 1; i <= vertex_num; i++)
        key[i] = INT_MAX, mstSet[i] = 0;

     key[1] = 0;     
     parent[1] = -1;
     for (int count = 1; count < vertex_num; count++)
     {
        int u = minKey(key, mstSet);

        mstSet[u] = 1;

        for (int v = 1; v <=vertex_num; v++)
          if (adj_mat[u][v] && mstSet[v] == 0 && adj_mat[u][v] <  key[v])
             {
                 parent[v]  = u;
                 key[v] = adj_mat[u][v];
     }


     }
     printMST(parent, vertex_num, adj_mat);
}

void mst_count_control(void)

{
	if((iter_mst>=1 && iter_mst<=count_mst+1) && stage_5==1 && mst_complete==1)
		iter_mst++;
}


//minimum spanning tree data finish





struct co_ordinates			//vertex co-ordinates

{
	double x;
	double y;
	int no;
}graph_nodes[MAX];


struct connections							//edges(v_x,v_y) edgese between vertex 'x' and vertex 'y'

{
	int  v_x;
	int  v_y;
	char cost[10];
}edges[MAX];


//number to string generator


char str_rev[100];

void num_to_string(int n)
{
char str[100];
int i=0;
while(n>0)
{
str[i++]=(n%10)+'0';
n/=10;
}
int len=i-1;
i=0;
while(len>=0)
{
str_rev[i++]=str[len--];
}
str_rev[i]='\0';

}


void Simulation_display(void)


{
		iClear();
		iSetcolor(0,0,255);
		iFilledRectangle(0,735,1366,100);

		iSetcolor(0,0,0);
		iText(350,750,"GRAPH ALGORITHM SIMULATION",GLUT_BITMAP_HELVETICA_18);

		iSetcolor(0,10,0);
		iFilledRectangle(280,590,400,50);
		
		iSetcolor(0,128,128);
		iFilledRectangle(300,600,400,50);
		
		iSetcolor(0,0,0);
		iText(350,615,"Breadth First Search",GLUT_BITMAP_HELVETICA_18);


		
		iSetcolor(0,10,0);
		iFilledRectangle(280,440,400,50);
		iSetcolor(0,128,128);
		iFilledRectangle(300,450,400,50);
		
		iSetcolor(0,0,0);
		iText(350,465,"Depth First Search",GLUT_BITMAP_HELVETICA_18);
		
		
		iSetcolor(0,10,0);
		iFilledRectangle(280,290,400,50);
		iSetcolor(0,128,128);
		iFilledRectangle(300,300,400,50);

		iSetcolor(0,0,0);
		iText(350,315,"Minimum Spanning Tree(Prim)",GLUT_BITMAP_HELVETICA_18);
		
		
		iSetcolor(0,10,0);
		iFilledRectangle(280,140,400,50);
		iSetcolor(0,128,128);
		iFilledRectangle(300,150,400,50);
		
		iSetcolor(0,0,0);
		iText(350,165,"Dijkstra's Shortest Path",GLUT_BITMAP_HELVETICA_18);



}


void Create_Display(void)


{

  iClear();
		iSetcolor(0,0,255);
		iFilledRectangle(0,0,160,1366);
		iSetcolor(10,255,20);
		iFilledRectangle(0,735,1366,100);
		iSetcolor(255,1,1);
		iText(300,750,"GRAPH ALGORITHM SIMULATION",GLUT_BITMAP_HELVETICA_18);

		iSetcolor(0,100,255);
		iRectangle(5,635,150,50);		//Create Graph
		iSetcolor(255,1,1);
		iText(7,650,"CREATE GRAPH",GLUT_BITMAP_HELVETICA_18);
		
	
		iSetcolor(0,100,255);
		iRectangle(5,100,100,50);   //Quit
		iSetcolor(255,1,1);
		iText(25,115,"QUIT",GLUT_BITMAP_HELVETICA_18);
}

/* 
	function iDraw() is called again and again by the system.
*/
void iDraw()
{
		/*_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _*/

	//printf("flag = %d\n",flag);
	//Main Display is shown and options are given to choose 1.Create A Graph , 2. Exit


	if(!image_flag)
	{	
		iSetcolor(0,0,0);
		iFilledRectangle(0,0,1366,100);
	}


	if(simulation_image)

	{
		Simulation_display();
	}




	if(image_flag)
		{ 
		Create_Display();
		}
	

	//this portion will not be displayed after a choice selection has been made


		/*_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _*/
	
	//printf("%d %d %d\n",stage_1,stage_2,stage_4);
	
	//stage-1 starts when stage_1==1, taking vertex_num from user

	if(stage_1)
	{
	
		iClear();
		iText_Field("Enter Vertex Number:");
		vertex_num=value;
	  if(vertex_num)
		  {
		  printf("Vertex num: %llu\n",vertex_num);
		  stage_1=0;
		  stage_2=1;
		  value=0;
	  }
	}


	
	//stage_1 finishes here , vertex_num has been taken , now stage_1==0 and stage_2==1


	/*_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _*/



	//now , stage_2==1 so stage_2 starts , value for edge_num is taken here
	

	if(stage_2)
	{
	
		iClear();

	iText_Field("Enter Edge Number:");
	  edge_num=value;
	  if(edge_num)
		  
	  {   printf("Edge num: %llu\n",edge_num);
		  stage_2=0;
		  flag=0;
		  value=0;
		  iClear();
		  stage_3=1;
	  }
	 
	}

	
	//stage_2 finishes here , edge_num has been taken , now stage_2==0 , stage_3==1

		/*_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _*/

	
	//this code_block is stage_3 , this is kept open , because the created nodes  are always shown 


		for(int i=1;i<=count_edge;i++)	
			{
				iSetcolor(0,0,1);
				iLine(graph_nodes[edges[i].v_x].x,graph_nodes[edges[i].v_x].y,graph_nodes[edges[i].v_y].x,graph_nodes[edges[i].v_y].y);
				iSetcolor(1,0,0);
				iText((graph_nodes[edges[i].v_x].x+graph_nodes[edges[i].v_y].x)/2,(graph_nodes[edges[i].v_x].y+graph_nodes[edges[i].v_y].y)/2,edges[i].cost,GLUT_BITMAP_HELVETICA_18);
		
			}



	
for(int i=1;i<=node_count;i++)
{       iSetcolor(259,259,259);
		iFilledCircle(graph_nodes[i].x,graph_nodes[i].y,15,200);
		iSetcolor(0,0,100);
		num_to_string(graph_nodes[i].no);
		iText(graph_nodes[i].x-25,graph_nodes[i].y,str_rev,GLUT_BITMAP_8_BY_13);   //node creation print
	//place your drawing codes here	
}     



//code for stage_3 finishes here , but this code block will run until the program has done it's job and it terminates 

		/*_ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _ _*/



//Stage_4 , is the pair-wise node selection part

      iSetcolor(0,0,1);
	  if(count_edge<=edge_num)
	  {
	if(selection_flag1>=0)
		{   iSetcolor(0,0,1);
			iCircle(graph_nodes[selection_flag1].x,graph_nodes[selection_flag1].y,12,200);		//node selection print
		}
	if(selection_flag2>=0)
	{	
		iCircle(graph_nodes[selection_flag2].x,graph_nodes[selection_flag2].y,12,200);
	  
	}
	  }
	
	//printf("Count edge==%d\n",count_edge);

		
	




	/*stage_4 cost taking portion*/

	if(stage_4==1)
	{
	
	 
		if(weighted)
		{
			iText_Field("Enter Cost For This Edge:");
			cost=value;

	  if(cost)
		  {
		printf("Cost = %d\n",cost);
		adj_mat[edges[count_edge].v_x][edges[count_edge].v_y]=cost;  //inserting cost to adj[i][j] and adj[j][i] indices
		adj_mat[edges[count_edge].v_y][edges[count_edge].v_x]=cost;
		num_to_string(cost);
		strcpy_s(edges[count_edge].cost,str_rev);
		iSetcolor(1,0,0);
				flag=1;
			  stage_4=0;
	
			  cost=0;
			  value=0;
			  count_edge++;
		
			 
	  }
		}

		else if(weighted==0)

		{
			//printf("%d %d %d\n\n",count_edge,edges[count_edge].v_x,edges[count_edge].v_y);
			adj_mat[edges[count_edge].v_x][edges[count_edge].v_y]=1;  //inserting cost to adj[i][j] and adj[j][i] indices
		adj_mat[edges[count_edge].v_y][edges[count_edge].v_x]=1;
				flag=1;
			  stage_4=0;
			count_edge++;
			  iClear();

		}

}
	if(count_edge==edge_num+1 && stage_4==0 && edge_num!=0)
	{
			if(source_choose_stage)
			{
			if(flag_dfs==1)
			{
				iText_Field("Enter Source Node:");
				source_D=value;
				if(source_D>0)
					{
						dfs(source_D);
						source_choose_stage=0;
						stage_5=1;
						iClear();
						value=0;
						stage_4=2;
				}
			}


			else if(flag_bfs==1)
			{

				iText_Field("Enter Source Node:");
				source_D=value;
				if(source_D>0)
					{
						bfs(source_D);
						source_choose_stage=0;
						stage_5=1;
						iClear();
						value=0;
						stage_4=2;
				}


			}

			else if(flag_dijkstra==1)
			 {


				if(!flag_D)
				{
				iText_Field("Enter Source Node:");
				source_D=value;
				printf("Destination node %d\n",source_D);
				if(source_D>0)
					flag_D=1;
				value=0;
				}
				
				else if(flag_D==1)
				{
				iClear();
				iText_Field("Enter Destination Node:");
				dest_D=value;
				printf("Destination node %d\n",dest_D);
				if(dest_D>0)
						flag_D=2;
			
				}

				else if(flag_D==2)

				{	iClear();
					Dijkstra(source_D,dest_D);
					source_choose_stage=0;
						stage_5=1;
						value=0;
						stage_4=2;
				}
			}


			else if(flag_mst==1)
				{primMST();
			stage_5=1;
			stage_4=2;
			}
		}
	}


	if(dijkstra_over)

			{


				if(iter_track!=vertex_num+1)
				{
				for(int i=1;i<=iter_track;i++)
			{
				
				if(track_path_D[i].node!=0 && track_path_D[i].prev_node!=0)
				{
					iSetcolor(1,0,0);
					iSetcolor(0,1,0);
					iLine(graph_nodes[track_path_D[i].node].x,graph_nodes[track_path_D[i].node].y,graph_nodes[track_path_D[i].prev_node].x,graph_nodes[track_path_D[i].prev_node].y);
					iFilledCircle(graph_nodes[track_path_D[i].node].x,graph_nodes[track_path_D[i].node].y,12);
					iFilledCircle(graph_nodes[track_path_D[i].prev_node].x,graph_nodes[track_path_D[i].prev_node].y,12);
					
				}

			}
				}
			


			}



	
	//stage_4 finishes here , cost has been taken , now stage_4==0 , flag=1 , ready for connecting the next pair of vertices :)

	if(stage_5)
	{

	if(flag_bfs)
	{
		if(count_bfs<iter_bfs)

		{
			iSetcolor(0,1,0);
			for(int i=1;i<=count_bfs;i++)
			{
			for(int j=1;j<=vertex_num;j++)
			{
				if(save_visited_bfs[i][j]!=0)
				iFilledCircle(graph_nodes[save_visited_bfs[i][j]].x,graph_nodes[save_visited_bfs[i][j]].y,12);

			}	
		}

					iSetcolor(1,0,0);
			for(int i=1;i<=count_bfs;i++)
			{
			for(int j=1;j<=vertex_num;j++)
			{
				if(black_mat[i][j]!=0)
				iFilledCircle(graph_nodes[black_mat[i][j]].x,graph_nodes[black_mat[i][j]].y,12);
			}
			}
		}

		if(count_bfs==iter_bfs)
			stage_5=0;
	}

	
else if(flag_dfs)	


	{		
		if(dfs_count<=iter_dfs)
		{
			iSetcolor(1,0,0);

			for(int i=1;i<=vertex_num;i++)
			{
				if(dfs_path[dfs_count][i]!=0)
				{iFilledCircle(graph_nodes[dfs_path[dfs_count][i]].x,graph_nodes[dfs_path[dfs_count][i]].y,12,200);				
				}
			}

			iSetcolor(0,1,0);
			for(int i=1;i<=dfs_count;i++)
			{
				iSetcolor(0,1,0);	
				iLine(graph_nodes[dfs_track[i].node].x,graph_nodes[dfs_track[i].node].y,graph_nodes[dfs_track[i].prev_node].x,graph_nodes[dfs_track[i].prev_node].y);
				iSetcolor(1,0,0);
				iFilledCircle(graph_nodes[dfs_track[i].node].x,graph_nodes[dfs_track[i].node].y,12);
				iFilledCircle(graph_nodes[dfs_track[i].prev_node].x,graph_nodes[dfs_track[i].prev_node].y,12);

			}

		}


		
		if(dfs_count==iter_dfs+1)
			stage_5=0;
			

	
	}


else if(flag_mst)


	{
		//printf("DEBUG");
	
	for(int i=1;i<=iter_mst+1;i++)
	{
	iSetcolor(0,1,0);
	iLine(graph_nodes[track_mst[i].node].x,graph_nodes[track_mst[i].node].y,graph_nodes[track_mst[i].prev_node].x,graph_nodes[track_mst[i].prev_node].y);
	iSetcolor(1,0,0);
	iFilledCircle(graph_nodes[track_mst[i].node].x,graph_nodes[track_mst[i].node].y,12);
	iFilledCircle(graph_nodes[track_mst[i].prev_node].x,graph_nodes[track_mst[i].prev_node].y,12);
	}

	if(iter_mst==count_mst+2)
		stage_5=0;

			}

	


else if(flag_dijkstra)


	{
	
		if(count_dijkstra<iter_dijkstra)
		{
		for(int i=1;i<=vertex_num;i++)
		{
			if(node_cost[count_dijkstra][i].color==GREY)
			    iSetcolor(1,0,0);
			else if(node_cost[count_dijkstra][i].color==BLACK)
				iSetcolor(0,1,0);
			iFilledCircle(graph_nodes[i].x,graph_nodes[i].y,12);
			
		}


		for(int i=1;i<=vertex_num;i++)
		{
				if(node_cost[count_dijkstra][i].cost==-1)
				{
					iText(graph_nodes[i].x,graph_nodes[i].y,"inf",GLUT_BITMAP_HELVETICA_10);
					iSetcolor(0,0,0);
				}
				else
				{
					num_to_string(node_cost[count_dijkstra][i].cost);
					iSetcolor(0,0,0);
					iText(graph_nodes[i].x,graph_nodes[i].y,str_rev,GLUT_BITMAP_HELVETICA_10);
				}
		
		}

	}

	
		if(count_dijkstra==iter_dijkstra)
		{

			stage_5=0;
			dijkstra_over=1;

		}


	
	}

}



			


	}
/* 
	function iMouseMove() is called when the user presses and drags the mouse.
	(mx, my) is the position where the mouse pointer is.
*/
void iMouseMove(int mx, int my)
{
	//place your codes here
}

/* 
	function iMouse() is called when the user presses/releases the mouse.
	(mx, my) is the position where the mouse pointer is.
*/


void iMouse(int button, int state, int mx, int my)
{



	//printf("selection values =%d %d\n",selection_flag1,selection_flag2);
	
	if(button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{	
		//IMPORTANT PORTION TO REMEMBER FOR THIS CODE BLOCK IS THE VALUES OF 'flag' variable , how it's changing to get the selection_flags
		
		
		/*flag = 0 , when plotting the nodes , when plotting is done it is changed to 1
		

		 flag=1 while choosing the 1st node

		  flag=2 while choosing the 2nd node

		  flag=3  2 nodes are selected

		*/
	
		//Stage_1 option  selection portion
		
		if(simulation_image && (stage_1==0))

		{



					if((mx>=300 && mx<=700) && (my>=600 && my<=650))  //bfs

			{
			//iSetcolor(0,0,1);
			//iFilledRectangle(50,500,500,50);
				simulation_image=0;  //Turning of the pic
				image_flag=1;
				flag_bfs=1;
				weighted=0;
		}
	
				else if((mx>=300 && mx<=700) && (my>=450 && my<=500))  //dfs

			{
				simulation_image=0;  //Turning of the pic
				image_flag=1;
				flag_dfs=1;
				weighted=0;
		}
			else if((mx>=400 && mx<=700) && (my>=300 && my<=350))  //mst

			{
				simulation_image=0;  //Turning of the pic
				image_flag=1;
				flag_mst=1;
				weighted=1;
		}
			else if((mx>=400 && mx<=700) && (my>=150 && my<=200))  //CREATE GRAPH

			{
				simulation_image=0;  //dijkstra
				image_flag=1;
				flag_dijkstra=1;
				weighted=1;
		}
	


		}




		else if(stage_1==0 && image_flag)
	
		{
	
			if((mx>=5 && mx<=155) && (my>=635 && my<=685))  //CREATE GRAPH

			{
			//iSetcolor(0,0,1);
			//iFilledRectangle(50,500,500,50);
		     image_flag=0;  //Turning of the pic
			 stage_1=1;

		}
				else if((mx>=5 && mx<=105) && (my>=100 && my<=150))  //QUIT
				exit(0);
		
		}


		
	
	
		
		//option selection ends here 


		else if(stage_3)
	{	
	
		
		

		//code for fetching co-ordinates of the nodes plotted

		if(flag==0)
		{
	if(node_count<=vertex_num)
	{
		graph_nodes[node_count].no=node_count;
		graph_nodes[node_count].x=mx;
		graph_nodes[node_count].y=my;
		node_count++;
	}
	if(node_count==vertex_num+1)
		flag=1;
		}
	
		

		
		//co-ordinate fetching is done for the current node 
	
		
		//Node selection code_snippet


		

	else if(flag==1)            //1st node selection

	{    for(int i=1;i<=node_count;i++)
		{
			if(((mx<15+graph_nodes[i].x) && (graph_nodes[i].x)-15<=mx) && ((my<15+graph_nodes[i].y) && (graph_nodes[i].y)-15<=my))
			{
				selection_flag1=i;
				flag=2;
				break;
			}
		}
	}


	//node selection code_snippet


	else if(flag==2)             //2nd node selection

	{
		for(int i=1;i<=vertex_num;i++)
		{
			if(((mx<15+graph_nodes[i].x) && (graph_nodes[i].x)-15<=mx) && ((my<15+graph_nodes[i].y) && (graph_nodes[i].y)-15<=my))
			{
				selection_flag2=i;
				flag=3;
			}
		}
}

				//code for pairwise node connection
	if(flag==3)   
{ 
		
	if(count_edge<=edge_num)    //if edge limit is not exceeded 
	{
		edges[count_edge].v_x=selection_flag1;
			edges[count_edge].v_y=selection_flag2;
		stage_4=1;
		
		
		}
	}


	}

	}
		//code for pairwise node connection ends here 


		//place your codes here	
	

	

	if(button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN)
	{
		//place your codes here	
	}
}



/*
	function iKeyboard() is called whenever the user hits a key in keyboard.
	key- holds the ASCII value of the key pressed.	
*/


void iKeyboard(unsigned char key)
{
	if(key == 'q')
	{
		exit(0);
	}
	
	//place your codes for other keys here
	if(key=='P')
	{
		adj_mat_print();
	}
		
//TEXT FIELD CODE START
		
	if(key=='\r')
	{	
			max_char=char_MAX;
			char_x=10;
			value=numgen();
			printf("Entered value is: %llu\n",value);
			char_top=-1;
		}
	if(char_top>=0 && key=='\b')
		{char_top-=1;
		char_x-=10;
	}

	if(max_char>0 )
	{   
			
		if('0'<=key && key<='9')
	{   
		stack[++char_top].num[0]=key;
		stack[char_top].pos=char_x+320;
		char_x+=10;
		max_char--;
	}

	
}

//TEXT FIELD CODE END


}

void adj_mat_print(void)

{
	for(int i=1;i<=vertex_num;i++)
	{
		
		for(int j=1;j<=vertex_num;j++)
			printf("%d\t",adj_mat[i][j]);
			printf("\n");

		}
}

void blink_control(void)

{
	char_flag=0;
}



unsigned long long int numgen(void)

{   unsigned long long int sum=0;
	for(int i=0;i<=char_top;i++)
		sum=sum*10+stack[i].num[0]-'0';
	
	return sum;
}



int main()
{   
	for(int i=1;i<=5;i++)
		{graph_nodes[i].x=0;
		graph_nodes[i].y=0;
}
	iSettimer(100,blink_control);
	iSettimer(2000,dfs_count_control);
	iSettimer(2000,bfs_count_control);
	iSettimer(5000,dijkstra_count_control);
	iSettimer(3000,dijkstra_track_control);
	iSettimer(3000,mst_count_control);
	//place your own initialization codes here. 	
	iInitialize(1366, 768, "Graph Algorithm Simulation");
	return 0;
}	