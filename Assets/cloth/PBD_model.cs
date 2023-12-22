using UnityEngine;
using System.Collections;

public class PBD_model: MonoBehaviour {

	float 		t= 0.0333f;	//每次模拟的时间长度
	float		damping= 0.99f; //摩擦系数，用来模拟空气阻力
	int[] 		E;	//每两个一组，按照边的长度顺序存储了对应的顶点下标
	float[] 	L;	//每根弹簧的原长
	Vector3[] 	V;	//每个顶点的速度


	// Use this for initialization
	void Start () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;

		//Resize the mesh.
		int n=21;
		Vector3[] X  	= new Vector3[n*n];	//网格中每个顶点的位置
		Vector2[] UV 	= new Vector2[n*n]; //每个顶点在纹理中的位置
		int[] T	= new int[(n-1)*(n-1)*6];  //每个三角形对应顶点的索引(在X[]的索引)
		for(int j=0; j<n; j++)
			for(int i=0; i<n; i++) {
				X[j*n+i] =new Vector3(5-10.0f*i/(n-1), 0, 5-10.0f*j/(n-1));
				UV[j*n+i]=new Vector3(i/(n-1.0f), j/(n-1.0f));
			}
		int t=0;
		for(int j=0; j<n-1; j++) 
			for(int i=0; i<n-1; i++) {
				T[t*6+0]=j*n+i;
				T[t*6+1]=j*n+i+1;
				T[t*6+2]=(j+1)*n+i+1;
				T[t*6+3]=j*n+i;
				T[t*6+4]=(j+1)*n+i+1;
				T[t*6+5]=(j+1)*n+i;
				t++;
			}
		mesh.vertices	= X;
		mesh.triangles	= T;
		mesh.uv 		= UV;
		mesh.RecalculateNormals ();

		//Construct the original edge list 建边
		int[] _E = new int[T.Length*2]; //每条边中的顶点的索引(此时还未去除重复边)
		for (int i=0; i<T.Length; i+=3) 
		{
			_E[i*2+0]=T[i+0];
			_E[i*2+1]=T[i+1];
			_E[i*2+2]=T[i+1];
			_E[i*2+3]=T[i+2];
			_E[i*2+4]=T[i+2];
			_E[i*2+5]=T[i+0];
		}
		//Reorder the original edge list
		for (int i=0; i<_E.Length; i+=2)
			if(_E[i] > _E[i + 1]) 
				Swap(ref _E[i], ref _E[i+1]);
		//Sort the original edge list using quicksort
		Quick_Sort (ref _E, 0, _E.Length/2-1);

		int e_number = 0;
		for (int i=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
				e_number++;

		E = new int[e_number * 2];	//去重后的边
		for (int i=0, e=0; i<_E.Length; i+=2)
			if (i == 0 || _E [i + 0] != _E [i - 2] || _E [i + 1] != _E [i - 1]) 
			{
				E[e*2+0]=_E [i + 0];
				E[e*2+1]=_E [i + 1];
				e++;
			}

		L = new float[E.Length/2];
		for (int e=0; e<E.Length/2; e++) 
		{
			int i = E[e*2+0];
			int j = E[e*2+1];
			L[e]=(X[i]-X[j]).magnitude;
		}

		V = new Vector3[X.Length];
		for (int i=0; i<X.Length; i++)
			V[i] = new Vector3 (0, 0, 0);
	}

	void Quick_Sort(ref int[] a, int l, int r)
	{
		int j;
		if(l<r)
		{
			j=Quick_Sort_Partition(ref a, l, r);
			Quick_Sort (ref a, l, j-1);
			Quick_Sort (ref a, j+1, r);
		}
	}

	int  Quick_Sort_Partition(ref int[] a, int l, int r)
	{
		int pivot_0, pivot_1, i, j;
		pivot_0 = a [l * 2 + 0];
		pivot_1 = a [l * 2 + 1];
		i = l;
		j = r + 1;
		while (true) 
		{
			do ++i; while( i<=r && (a[i*2]<pivot_0 || a[i*2]==pivot_0 && a[i*2+1]<=pivot_1));
			do --j; while(  a[j*2]>pivot_0 || a[j*2]==pivot_0 && a[j*2+1]> pivot_1);
			if(i>=j)	break;
			Swap(ref a[i*2], ref a[j*2]);
			Swap(ref a[i*2+1], ref a[j*2+1]);
		}
		Swap (ref a [l * 2 + 0], ref a [j * 2 + 0]);
		Swap (ref a [l * 2 + 1], ref a [j * 2 + 1]);
		return j;
	}

	void Swap(ref int a, ref int b)
	{
		int temp = a;
		a = b;
		b = temp;
	}

	void Strain_Limiting()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] vertices = mesh.vertices;

		//Apply PBD here.
		Vector3[] sum_X = new Vector3[vertices.Length];	//顶点的位置更新和
		float[] sum_n = new float[vertices.Length];		//顶点的更新次数

		for (int i = 0; i < vertices.Length; i++) {	//先算一次更新
			sum_X[i] = vertices[i] * 0.2f;
			sum_n[i] = 0.2f;
		}

		for (int e = 0; e < E.Length / 2; e++) {
			int i = E[e << 1];
			int j = E[e << 1 | 1];
			Vector3 A = 0.5f * (vertices[i] + vertices[j]);
			Vector3 B = 0.5f * L[e] * (vertices[i] - vertices[j]).normalized;

			sum_X[i] += A + B;
			sum_X[j] += A - B;
			sum_n[i]++;
			sum_n[j]++;
		}
		//如果我们把更新sum_x数组的那一段，不使用中间变量A和B，而是写成一坨会导致整个程序运行效率直线下降，fps降低了至少30%左右。
		//在面对这些大型公式的时候，重复部分还是要尽量单独提出来做一个新的中间变量，防止重复计算使得程序效率降低。
		
		//先更新速度v再更新x，否则v就更新不了了，因为后面一串的计算结果直接等于0
		for (int i = 0; i < vertices.Length; i++) {
			if (i == 0 || i == 20) continue;
			Vector3 A = sum_X[i] / sum_n[i];
			V[i] += (A - vertices[i]) / t;
			vertices[i] = A;
		}
		
		mesh.vertices = vertices;
	}

	void Collision_Handling()
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;
		
		//For every vertex, detect collision and apply impulse if needed.
		GameObject sphere = GameObject.Find("Sphere");

		Vector3 center = sphere.transform.TransformPoint(new Vector3(0, 0, 0));
		float radius = 2.7f;

		for (int i = 0; i < X.Length; i++) {
			if (i == 0 || i == 20)	continue;

			Vector3 d = X[i] - center;
			if (d.magnitude < radius) {
				Vector3 A = center + radius * d.normalized;
				V[i] += (A - X[i]) / t;
				X[i] = A;
			}
		}
		
		mesh.vertices = X;
	}

	// Update is called once per frame
	void Update () 
	{
		Mesh mesh = GetComponent<MeshFilter> ().mesh;
		Vector3[] X = mesh.vertices;

		for(int i=0; i<X.Length; i++)
		{
			if(i==0 || i==20)	continue;
			V[i] += t * new Vector3(0, -9.8f, 0);
			V[i] *= damping;
			X[i] += t * V[i];
		}
		mesh.vertices = X;

		for (int l = 0; l < 32; l ++)
			Strain_Limiting ();

		Collision_Handling ();

		mesh.RecalculateNormals ();

	}
}

