using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using Unity.VisualScripting;

public class FVM : MonoBehaviour
{
	float dt 			= 0.003f;
    float mass 			= 1;
	float stiffness_0	= 20000.0f;
    float stiffness_1 	= 5000.0f;
    float damp			= 0.999f;

	int[] 		Tet;		//模拟四面体
	int tet_number;			//The number of tetrahedra

	Vector3[] 	Force;		//每个点所受到的力的总和
	Vector3[] 	V;			//每个点的速度
	Vector3[] 	X;			//每个点的位置
	int number;				//The number of vertices

	float FloorYPos;		//平面的y轴坐标，处理碰撞用

	Matrix4x4[] inv_Dm;

	//For Laplacian smoothing.
	Vector3[]   V_sum;		//存储每个顶点的邻接顶点速度和
	int[]		V_num;		//存储邻接顶点的个数
	
	Boolean chooseSVD = false;
	SVD svd = new SVD();

    // Start is called before the first frame update
    void Start()
    {
    	// FILO IO: Read the house model from files.
    	// The model is from Jonathan Schewchuk's Stellar lib.
    	{
    		string fileContent = File.ReadAllText("Assets/FVM/house2.ele");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
		    
		    //Tet_number是四面体个数。每个四面体有四个顶点，把顶点的index保存在Tet里面。所以Tet是长度为tet_number * 4的int数组W
    		tet_number=int.Parse(Strings[0]);
        	Tet = new int[tet_number * 4];

    		for(int tet=0; tet<tet_number; tet++)
    		{
				Tet[tet*4+0]=int.Parse(Strings[tet*5+4])-1;
				Tet[tet*4+1]=int.Parse(Strings[tet*5+5])-1;
				Tet[tet*4+2]=int.Parse(Strings[tet*5+6])-1;
				Tet[tet*4+3]=int.Parse(Strings[tet*5+7])-1;
			}
    	}
    	{
			string fileContent = File.ReadAllText("Assets/FVM/house2.node");
    		string[] Strings = fileContent.Split(new char[]{' ', '\t', '\r', '\n'}, StringSplitOptions.RemoveEmptyEntries);
		    
		    //存一下位置
    		number = int.Parse(Strings[0]);
    		X = new Vector3[number];
       		for(int i=0; i<number; i++)
       		{
       			X[i].x=float.Parse(Strings[i*5+5])*0.4f;
       			X[i].y=float.Parse(Strings[i*5+6])*0.4f;
       			X[i].z=float.Parse(Strings[i*5+7])*0.4f;
       		}
    		//Centralize the model.			中心化模型
	    	Vector3 center=Vector3.zero;
	    	for(int i=0; i<number; i++)		center+=X[i];
	    	center=center/number;			//得到模型的几何中心
	    	for(int i=0; i<number; i++)
	    	{
	    		X[i]-=center;				//从每个顶点中减去几何中心，这样做将模型移动到原点
	    		float temp=X[i].y;			//交换每个顶点的Y和Z坐标。转换左右手坐标系
	    		X[i].y=X[i].z;
	    		X[i].z=temp;
	    	}
		}
        /*tet_number=1;
        Tet = new int[tet_number*4];
        Tet[0]=0;
        Tet[1]=1;
        Tet[2]=2;
        Tet[3]=3;

        number=4;
        X = new Vector3[number];
        V = new Vector3[number];
        Force = new Vector3[number];
        X[0]= new Vector3(0, 0, 0);
        X[1]= new Vector3(1, 0, 0);
        X[2]= new Vector3(0, 1, 0);
        X[3]= new Vector3(0, 0, 1);*/


        //Create triangle mesh.
        //生成Mesh，一共Vertices有tet_number * 12那么长，因为每个四面体四个顶点，每个顶点有三个坐标。
       	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];

        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];

        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }

        int[] triangles = new int[tet_number*12];
        for(int t=0; t<tet_number*4; t++)
        {
        	triangles[t*3+0]=t*3+0;
        	triangles[t*3+1]=t*3+1;
        	triangles[t*3+2]=t*3+2;
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.triangles = triangles;
		mesh.RecalculateNormals ();


		V 	  = new Vector3[number];
        Force = new Vector3[number];
        V_sum = new Vector3[number];
        V_num = new int[number];
        
        //取一下平面的y坐标
        GameObject floor = GameObject.Find("Floor");
        FloorYPos = floor.transform.position.y;

		//TODO: Need to allocate and assign inv_Dm	求Dm（X1,X2,X3）的逆  形变前的坐标X
		inv_Dm = new Matrix4x4[tet_number];
		for (int i = 0; i < tet_number; i++)
			inv_Dm[i] = Build_Edge_Matrix(i).inverse;
    }
	
    //为所有四面体生成对应的边矩阵
    Matrix4x4 Build_Edge_Matrix(int tet)
    {
    	Matrix4x4 ret=Matrix4x4.zero;
    	//TODO: Need to build edge matrix here.
	    Vector4 x10 = X[Tet[tet * 4 + 1]] - X[Tet[tet << 2]];
	    Vector4 x20 = X[Tet[tet * 4 + 2]] - X[Tet[tet << 2]];
	    Vector4 x30 = X[Tet[tet * 4 + 3]] - X[Tet[tet << 2]];
	    
	    ret.SetColumn(0, x10);
	    ret.SetColumn(1, x20);
	    ret.SetColumn(2, x30);
	    ret.SetColumn(3, new Vector4(0, 0, 0, 1));
		return ret;
    }
	//在机器学习中，拉普拉斯平滑是为了防止某一种情况出现0，在统计的时候，给每个情况的样本量都加上1。
	//例如：对于一个事件有A、B、C三种状态，进行1000次观察之后，三种情况分别出现的次数为0，990，10。则对应K1的概率为0，0.99，0.01。
	//我们不希望让A事件得到0概率，我们就给每个情况出现的次数加1，也就是1，991，11，来进行平滑。
	//所以经过拉普拉斯平滑之后的数据是：1/1003 = 0.001，991/1003=0.988，11/1003=0.011。
	
	//在这里的模拟中，我们要采用拉普拉斯平滑的话，对于每一个顶点，我们需要计算其相邻顶点的平均速度，然后根据相邻顶点平均速度来更新原本的顶点速度。
	//由于代码内部没有提供邻接矩阵，所以我们需要按照四面体来进行遍历。思路类似于根据面法线算顶点法线。
	//对于每个四面体，求它所有顶点的速度并且加到V_sum数组里即可。这样就可以绕过点的邻接矩阵来进行运算了
	void Smooth_V() {
		for (int i = 0; i < number; i ++ ) {
			V_sum[i] = new Vector3(0, 0, 0);
			V_num[i] = 0;
		}
		
		for (int tet = 0; tet < tet_number; tet ++ ) {
			Vector3 sum = V[Tet[tet * 4 + 0]] + V[Tet[tet * 4 + 1]] + V[Tet[tet * 4 + 2]] + V[Tet[tet * 4 + 3]];
			V_sum[Tet[tet * 4 + 0]] += sum;
			V_sum[Tet[tet * 4 + 1]] += sum;
			V_sum[Tet[tet * 4 + 2]] += sum;
			V_sum[Tet[tet * 4 + 3]] += sum;
			V_num[Tet[tet * 4 + 0]] += 4;
			V_num[Tet[tet * 4 + 1]] += 4;
			V_num[Tet[tet * 4 + 2]] += 4;
			V_num[Tet[tet * 4 + 3]] += 4;
		}
		//blend 混合速度
		for (int i = 0; i < number; i ++ )
			V[i] = 0.9f * V[i] + 0.1f * V_sum[i] / V_num[i];
	}
	
    void _Update()
    {
    	// Jump up.
		if(Input.GetKeyDown(KeyCode.Space))
    	{
    		for(int i=0; i<number; i++)
    			V[i].y+=0.2f;
    	}
		//TODO: Add gravity to Force.
    	for(int i=0 ;i<number; i++) {
		    Force[i] = new Vector3(0, -9.8f * mass, 0);
	    }

    	for(int tet=0; tet<tet_number; tet++)
    	{
    		//TODO: Deformation Gradient
		    Matrix4x4 F = Build_Edge_Matrix(tet) * inv_Dm[tet];
		    Matrix4x4 P = Matrix4x4.zero;
		    if (chooseSVD) {
			    Matrix4x4 U = Matrix4x4.zero;
			    Matrix4x4 S = Matrix4x4.zero;
			    Matrix4x4 V = Matrix4x4.zero;

			    svd.svd(F, ref U, ref S, ref V);
			    
			    float lambda0 = S[0, 0];
			    float lambda1 = S[1, 1];
			    float lambda2 = S[2, 2];

			    float Ic = lambda0 * lambda0 + lambda1 * lambda1 + lambda2 * lambda2;

			    float dWdIc = 0.25f * stiffness_0 * (Ic - 3f) - 0.5f * stiffness_1;
			    float dWdIIc = 0.25f * stiffness_1;
			    float dIcdlambda0 = 2f * lambda0;
			    float dIcdlambda1 = 2f * lambda1;
			    float dIcdlambda2 = 2f * lambda2;
			    float dIIcdlambda0 = 4f * lambda0 * lambda0 * lambda0;
			    float dIIcdlambda1 = 4f * lambda1 * lambda1 * lambda1;
			    float dIIcdlambda2 = 4f * lambda2 * lambda2 * lambda2;
			    float dWd0 = dWdIc * dIcdlambda0 + dWdIIc * dIIcdlambda0;
			    float dWd1 = dWdIc * dIcdlambda1 + dWdIIc * dIIcdlambda1;
			    float dWd2 = dWdIc * dIcdlambda2 + dWdIIc * dIIcdlambda2;

			    Matrix4x4 diag = Matrix4x4.zero;
			    diag[0, 0] = dWd0;
			    diag[1, 1] = dWd1;
			    diag[2, 2] = dWd2;
			    diag[3, 3] = 1;
			    P = U * diag * V.transpose;
		    }
		    else {
			    //TODO: Green Strain
			    Matrix4x4 G = F.transpose * F;
			    G[0, 0]--;
			    G[1, 1]--;
			    G[2, 2]--;
			    for (int i = 0; i < 3; i ++ )
			    for (int j = 0; j < 3; j ++)
				    G[i, j] *= 0.5f;
			    //TODO: Second PK Stress	第二应力用stvk模型 S = 2 μ G + λ trace(G) I
			    Matrix4x4 S = Matrix4x4.zero;
			    for (int i = 0; i < 3; i ++ )
			    for (int j = 0; j < 3; j++)
				    S[i, j] = 2 * stiffness_1 * G[i, j];
			    float A = stiffness_0 * (G[0, 0] + G[1, 1] + G[2, 2]);
			    S[0, 0] += A;
			    S[1, 1] += A;
			    S[2, 2] += A;
			    S[3, 3] = 1;
			    //TODO: Elastic Force
			    P = F * S;
		    }
		    Matrix4x4 forces = P * inv_Dm[tet].transpose;
		    float B = -1 / (inv_Dm[tet].determinant * 6);
		    for (int i = 0; i < 3; i ++ )
				for (int j = 0; j < 3; j ++)
					forces[i, j] *= B;
		    Force[Tet[tet * 4]] -= (Vector3)(forces.GetColumn(0) + forces.GetColumn(1) + forces.GetColumn(2));	//f0 = -f1-f2-f3
		    Force[Tet[tet * 4 + 1]] += (Vector3)forces.GetColumn(0);	//f1
		    Force[Tet[tet * 4 + 2]] += (Vector3)forces.GetColumn(1);	//f2
		    Force[Tet[tet * 4 + 3]] += (Vector3)forces.GetColumn(2);	//f3
	    }
		
	    //拉普拉斯平滑 Laplacian smoothing
	    Smooth_V();
	    
	    //更新位置和速度，处理碰撞
    	for(int i = 0; i < number; i++)
    	{
    		//TODO: Update X and V here.
		    V[i] = (V[i] + dt * Force[i] / mass) * damp;
		    X[i] = X[i] + dt * V[i];
		    //TODO: (Particle) collision with floor.
		    if (X[i].y < FloorYPos) {
			    V[i].y += (FloorYPos - X[i].y) / dt;
			    X[i].y = FloorYPos;
		    }
	    }
    }

    // Update is called once per frame
    void Update()
    {
    	for(int l=0; l<10; l++)
    		 _Update();

    	// Dump the vertex array for rendering.
    	Vector3[] vertices = new Vector3[tet_number*12];
        int vertex_number=0;
        for(int tet=0; tet<tet_number; tet++)
        {
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+0]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        	vertices[vertex_number++]=X[Tet[tet*4+1]];
        	vertices[vertex_number++]=X[Tet[tet*4+2]];
        	vertices[vertex_number++]=X[Tet[tet*4+3]];
        }
        Mesh mesh = GetComponent<MeshFilter> ().mesh;
		mesh.vertices  = vertices;
		mesh.RecalculateNormals ();
    }
}
