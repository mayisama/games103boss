using System;
using UnityEngine;
using System.Collections;
using Unity.VisualScripting;

public class Rigid_Bunny : MonoBehaviour 
{
	bool launched 		= false;
	float dt 			= 0.015f;  //步长
	Vector3 v 			= new Vector3(0, 0, 0);	// velocity
	Vector3 w 			= new Vector3(0, 0, 0);	// angular velocity
	
	float mass;									// mass
	Matrix4x4 I_ref;							// reference inertia  转动惯量

	float linear_decay	= 0.999f;				// for velocity decay
	float angular_decay	= 0.98f;				
	float restitution 	= 0.5f;					// for collision 弹性系数
	float friction = 0.5f;					// 摩擦系数

	private Vector3 gravity = new Vector3(0f, -9.8f, 0f);
	
	// Use this for initialization
	void Start () 
	{		
		Mesh mesh = GetComponent<MeshFilter>().mesh;
		Vector3[] vertices = mesh.vertices;

		float m=1;
		mass=0;
		for (int i=0; i<vertices.Length; i++) 
		{
			mass += m;
			float diag=m*vertices[i].sqrMagnitude;
			I_ref[0, 0]+=diag;
			I_ref[1, 1]+=diag;
			I_ref[2, 2]+=diag;
			I_ref[0, 0]-=m*vertices[i][0]*vertices[i][0];
			I_ref[0, 1]-=m*vertices[i][0]*vertices[i][1];
			I_ref[0, 2]-=m*vertices[i][0]*vertices[i][2];
			I_ref[1, 0]-=m*vertices[i][1]*vertices[i][0];
			I_ref[1, 1]-=m*vertices[i][1]*vertices[i][1];
			I_ref[1, 2]-=m*vertices[i][1]*vertices[i][2];
			I_ref[2, 0]-=m*vertices[i][2]*vertices[i][0];
			I_ref[2, 1]-=m*vertices[i][2]*vertices[i][1];
			I_ref[2, 2]-=m*vertices[i][2]*vertices[i][2];
		}
		I_ref [3, 3] = 1;
	}
	
	Matrix4x4 Get_Cross_Matrix(Vector3 a)
	{
		//Get the cross product matrix of vector a
		Matrix4x4 A = Matrix4x4.zero;
		A [0, 0] = 0; 
		A [0, 1] = -a [2]; 
		A [0, 2] = a [1]; 
		A [1, 0] = a [2]; 
		A [1, 1] = 0; 
		A [1, 2] = -a [0]; 
		A [2, 0] = -a [1]; 
		A [2, 1] = a [0]; 
		A [2, 2] = 0; 
		A [3, 3] = 1;
		return A;
	}
	
	private Matrix4x4 Matrix_Substraction(Matrix4x4 A, Matrix4x4 B)
	{
		for (int i = 0; i < 4; i ++ )
			for (int j = 0; j < 4; j ++ )
				A[i, j] -= B[i, j];
		return A;
	}

	private Matrix4x4 Matrix_Multiply_float(Matrix4x4 A, float b)
	{
		for (int i = 0; i < 4; i ++ )
			for (int j = 0; j < 4; j ++ )
				A[i, j] *= b;
		return A;
	}
	
	// In this function, update v and w by the impulse due to the collision with
	//a plane <P, N>   P 为该平面上的一个点，N为该平面的法线
	void Collision_Impulse(Vector3 P, Vector3 N)
	{
		//1.获取物体的每一个顶点(局部坐标)
		Vector3[] vertices = GetComponent<MeshFilter>().mesh.vertices;
		//2.得到每一个顶点的全局坐标旋转矩阵R,和平移向量
		Matrix4x4 R = Matrix4x4.Rotate(transform.rotation);		//旋转矩阵
		Vector3 x = transform.position;							//平移向量
		
		Vector3 sum_ri = new Vector3(0, 0, 0);			//碰撞点
		int sum = 0;											//碰撞点数量

		for (int i = 0; i < vertices.Length; i++)
		{
			//3.计算每个顶点到该表面的距离sdf(xi)
			Vector3 Rri = R.MultiplyVector(vertices[i]);
			Vector3 xi = x + Rri;
			if (Vector3.Dot(xi - P, N) < 0f)
			{
				Vector3 vi = v + Vector3.Cross(w, Rri);
				if (Vector3.Dot(vi, N) < 0f)
				{
					sum_ri += Rri;
					sum++;
				}
			}
		}

		if (sum == 0) return;

		Matrix4x4 inv_I = R * I_ref.inverse * R.transpose;	//转动惯量的逆（全局）
		Vector3 r_c = sum_ri / sum;
		Vector3 v_c = v + Vector3.Cross(w, r_c);
		if (Math.Abs(v_c.y + 9.8f*dt) < 0.01) restitution = 0;
		
		//更新V_i(new)
		Vector3 v_N = Vector3.Dot(v_c, N) * N;
		Vector3 v_T = v_c - v_N;
		float a = Math.Max(1.0f - friction * (1.0f + restitution) * v_N.magnitude / v_T.magnitude, 0f);
		Vector3 v_new = -1.0f * restitution * v_N + a * v_T;
		
		//计算冲量J
		Matrix4x4 Rstar = Get_Cross_Matrix(r_c);
		Matrix4x4 K = Matrix_Substraction(Matrix_Multiply_float(Matrix4x4.identity, 1.0f / mass), Rstar * inv_I * Rstar);
		Vector3 J = K.inverse * (v_new - v_c);
		
		//更新v，w
		v += 1.0f / mass * J;
		w += inv_I.MultiplyVector(Vector3.Cross(r_c, J));
	}

	// Update is called once per frame
	void Update () 
	{
		//Game Control
		if(Input.GetKey("r"))
		{
			transform.position = new Vector3 (0, 0.6f, 0);
			restitution = 0.5f;
			launched=false;
		}
		if(Input.GetKey("l"))
		{
			v = new Vector3 (5, 2, 0);
			launched=true;
		}

		// Part I: Update velocities
		v += dt * gravity;
		v *= linear_decay;
		w *= angular_decay;

		// Part II: Collision Impulse
		Collision_Impulse(new Vector3(0, 0.01f, 0), new Vector3(0, 1, 0));
		Collision_Impulse(new Vector3(2, 0, 0), new Vector3(-1, 0, 0));

		// Part III: Update position & orientation
		//Update linear status
		Vector3 x    = transform.position;
		if (launched) x += dt * v;
		
		//Update angular status
		Quaternion q = transform.rotation;
		Quaternion wq = new Quaternion(w.x, w.y, w.z, 0f);
		Quaternion temp_q = wq * q;
		q.x += 0.5f * dt * temp_q.x;
		q.y += 0.5f * dt * temp_q.y;
		q.z += 0.5f * dt * temp_q.z;
		q.w += 0.5f * dt * temp_q.w;
		
		
		// Part IV: Assign to the object
		transform.position = x;
		transform.rotation = q;
	}
}