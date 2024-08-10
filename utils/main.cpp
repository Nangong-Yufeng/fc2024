
// #include <bits/stdc++.h>
// using namespace std;
// typedef struct Quat{
// 	float s;         //scalar
// 	float v[3];		 //vector
// }QUAT;
 
// // QUAT Slerp_Inter(QUAT *Qs, QUAT *Qe, float lambda);
// // QUAT Squad(QUAT *Qi, QUAT *Si, QUAT *Si_1, QUAT *Qi_1, double t);
// // void GetCtlPoint(QUAT Qn[], int n, QUAT Sn[4]);

// //四元数相加或相减
// QUAT Quat_Add(QUAT *q1, QUAT *q2)
// {
// 	QUAT Q;
// 	Q.s    = q1->s+q2->s;
// 	Q.v[0] = q1->v[0]+q2->v[0];
// 	Q.v[1] = q1->v[1]+q2->v[1];
// 	Q.v[2] = q1->v[2]+q2->v[2];
 
// 	return Q;
// }
 

// //四元数标量乘法
// QUAT Quat_Smupltipy(QUAT *Q, double scalar)
// {
// 	QUAT q;
// 	q.s = Q->s*scalar;
// 	q.v[0] = Q->v[0]*scalar;
// 	q.v[1] = Q->v[1]*scalar;
// 	q.v[2] = Q->v[2]*scalar;
// 	return q;
// }
 
// //四元数内积或点积
// double Quat_Dot(QUAT *Q1, QUAT *Q2)
// {
// 	return (Q1->s*Q2->s + Q1->v[0]*Q2->v[0] + Q1->v[1]*Q2->v[1] + Q1->v[2]*Q2->v[2]);
// }

// //四元数外积或叉积
// QUAT Quat_Product(QUAT *q1, QUAT *q2)
// {
// 	QUAT Q;
// 	Q.s    = q1->s*q2->s    - q1->v[0]*q2->v[0] - q1->v[1]*q2->v[1] - q1->v[2]*q2->v[2] ;
// 	Q.v[0] = q1->v[0]*q2->s + q1->s*q2->v[0]    - q1->v[2]*q2->v[1] + q1->v[1]*q2->v[2] ;
// 	Q.v[1] = q1->v[1]*q2->s + q1->v[2]*q2->v[0] + q1->s*q2->v[1]    - q1->v[0]*q2->v[2] ;
// 	Q.v[2] = q1->v[2]*q2->s - q1->v[1]*q2->v[0] + q1->v[0]*q2->v[1] + q1->s*q2->v[2] ;
 
// 	return Q;
// }

// //四元数共轭
// QUAT Quat_Conj(QUAT *Q)
// {
// 	QUAT q;
// 	q.s = Q->s;
// 	q.v[0] = -Q->v[0];
// 	q.v[1] = -Q->v[1];
// 	q.v[2] = -Q->v[2];
// 	return q;
// }
 
// //四元数取反
// QUAT Quat_Reverse(QUAT *Q)
// {
// 	QUAT q;
// 	q.s = -Q->s;
// 	q.v[0] = -Q->v[0];
// 	q.v[1] = -Q->v[1];
// 	q.v[2] = -Q->v[2];
// 	return q;
// }
 
// //四元数模长
// double Quat_Norm(QUAT *q)
// {
// 	return (sqrt(q->s*q->s+
// 		         q->v[0]*q->v[0]+
// 		         q->v[1]*q->v[1]+
// 		         q->v[2]*q->v[2]));
// }

// //四元数对数运算
// QUAT Quat_Log(QUAT *q)
// {
//     //四元数求对数
//     // log(q)=[0, θv]
//     double sina = sqrt(q->v[0]*q->v[0]+q->v[1]*q->v[1]+q->v[2]*q->v[2]);
//     double cosa = q->s;
//     double theta = atan2(sina,cosa);
//     QUAT Q;
//     //当sina很小时，不能作分子，用theta代替sin(theta)
//     if(cosa > 0.9995){
//         Q = *q;
//     }
//     else{ 
//         Q = Quat_Smupltipy(q, theta/sina);
//     }
//     Q.s=0;
//     return Q;
// }
 
// //四元数指数运算
// QUAT Quat_Exp(QUAT *q)
// {
// 	//exp(q)=[cosθ,sinθv]
// 	//求四元数的指数
// 	double theta = sqrt(q->v[0]*q->v[0]+q->v[1]*q->v[1]+q->v[2]*q->v[2]);
// 	double cosa = cos(theta);
//     QUAT Q;
// 	//当sina很小时，不能作分子，用theta代替sin(theta)
// 	if(cosa > 0.9995){
// 		Q = *q;
// 	}
// 	else{
// 		Q = Quat_Smupltipy(q, sin(theta)/theta);
// 	}
// 	Q.s = cosa;
 
// 	return Q;
// }
 
// //计算控制点，形参n是控制点数，跟姿态个数相同
// void GetCtlPoint(QUAT Qn[], int n, QUAT Sn[])
// {
//     Sn[0] = Qn[0];        //第一个控制点和最后一个控制点无法由公式获取
//     Sn[n-1] = Qn[n-1];    //因此设置为与第一个插值点和最后一个插值点相同，对整个曲线影响不大
// 	QUAT qi,qi_conj,qi_m1,qi_m1_conj,qi_a1;
// 	QUAT m0,m1,m0_log,m1_log,m_log_sum,k,k_exp;
// 	int i = 0;
//     for(i = 1; i < n-1; i++){
//         qi = Qn[i];
//         qi_m1 = Qn[i-1];
//         qi_a1 = Qn[i+1];
//         if(Quat_Dot(&qi, &qi_m1)<0)  qi_m1 = Quat_Reverse(&qi_m1);
//         if(Quat_Dot(&qi, &qi_a1)<0)  qi_a1 = Quat_Reverse(&qi_a1);
//         qi_conj = Quat_Conj(&qi);
//         qi_m1_conj=Quat_Conj(&qi_m1);
//         m0 = Quat_Product(&qi_m1_conj, &qi);
//         m1 = Quat_Product(&qi_conj, &qi_a1);
 
// 		//k = (log(m0)-log(m1))/4;
// 		m0_log = Quat_Log(&m0);
// 		m1_log = Quat_Log(&m1);
// 		m_log_sum = Quat_Add(&m0_log, &Quat_Reverse(&m1_log));
// 		k = Quat_Smupltipy(&m_log_sum, 1/4);
// 		k_exp = Quat_Exp(&k);
//         Sn[i] = Quat_Product(&qi,&k_exp);
// 	}
// }
#include <bits/stdc++.h>
struct Node{
    int val,size,pri;
    Node *lchild,*rchild;
    Node(int v):val(v),size(1),pri(rand()),lchild(nullptr),rchild(nullptr){}
    void update(){
        size = 1;
        if (lchild!=nullptr) size += lchild->size;
        if (rchild!=nullptr) size += rchild->size;
    }
};
struct Treap{
    Node *root;
    Treap():root(nullptr){}
    void split(Node* p, int val, Node* &l, Node* &r){
        if (p == nullptr) {l = nullptr; r = nullptr; return;}
        if (p->val <= val) l = p, split(p->rchild, val, l->rchild, r);
        else r = p, split(p->lchild, val, l, r->lchild);
        p->update();
    }
    Node* merge(Node* sm, Node* bg){
        if (sm == nullptr) return bg;
        if (bg == nullptr) return sm;

        if (sm->pri < bg->pri){
            sm->rchild = merge(sm->rchild, bg);
            sm->update();
            return sm;
        }else {
            bg->lchild = merge(sm, bg->lchild);
            bg->update();
            return bg;
        }
    }

    void insert(int val){
        Node *l, *r;
        split(root, val, l, r);
        l = merge(l, new Node(val));
        root = merge(l, r);
    }
    void erase(int val){
        Node *l, *r, *mid;
        split(root, val, l, r);
        split(l, val-1, l, mid);
        Node* tmp = nullptr;
        if (mid != nullptr) tmp = merge(mid->lchild, mid->rchild);
        delete mid;
        l = merge(l, tmp);
        root = merge(l, r);
    }
    void get_rank(int val){
        Node *l, *r, *mid;
        split(root, val, l, r);
        split(l, val-1, l, mid);
        if (l!=nullptr) std::cout << l->size + 1 << std::endl;
        else std::cout << 1 << std::endl;
        root = merge(merge(l,mid), r);
    }
};
int main(){
    auto t = Treap();
    t.insert(1);
    t.insert(2);
    t.insert(3);
    t.insert(4);
    t.insert(5);
    t.insert(6);
    t.insert(7);
    t.insert(8);
    t.insert(9);
    t.insert(10);
    t.get_rank(7);   
    return 0;
}
// //四元数球面线性插值
// QUAT Slerp_Inter(QUAT *Qs, QUAT *Qe, float lambda)
// {
// 		float cosa = Qs->s*Qe->s + Qs->v[0]*Qe->v[0] + Qs->v[1]*Qe->v[1] + Qs->v[2]*Qe->v[2];
// 		float k0, k1;
// 		QUAT Qt;
// 		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
//     // the shorter path. Fix by reversing one quaternion.
// 		//q与-q实际上对应的是同一个旋转（double cover），为了得到最短路径，
// 		//插补之前应该判断两个四元数的角度，钝角则反转其中一个四元数
// 		if(cosa < 0){
// 			cosa = -cosa;
// 			Qe->s = -Qe->s;
// 			Qe->v[0] = -Qe->v[0];
// 			Qe->v[1] = -Qe->v[1];
// 			Qe->v[2] = -Qe->v[2];
// 		}
 
// 		// If the inputs are too close for comfort, linearly interpolate 
// 		//这里使用的是Lerp，使用Nlerp可能误差更小
// 		if(cosa > 0.9995f){
// 			k0 = 1.0f - lambda;
// 			k1 = lambda;
// 		}
// 		else{
// 			float sina = sqrt(1.0f - cosa*cosa);
// 			float a = atan2(sina, cosa);
// 			k0 = sin((1.0f - lambda)*a) / sina;
// 			k1 = sin(lambda*a) / sina;
// 		}
 
// 		Qt.s 	  = Qs->s*k0 + Qe->s*k1;
// 		Qt.v[0] = Qs->v[0]*k0 + Qe->v[0]*k1;
// 		Qt.v[1] = Qs->v[1]*k0 + Qe->v[1]*k1;
// 		Qt.v[2] = Qs->v[2]*k0 + Qe->v[2]*k1;
// 		return Qt;
// }
 
// //squad姿态插值
// QUAT Squad(QUAT *Qi, QUAT *Si, QUAT *Si_1, QUAT *Qi_1, double t)
// {
// 	QUAT k1 = Slerp_Inter(Qi,Qi_1,t);
// 	QUAT k2 = Slerp_Inter(Si,Si_1,t);
// 	return Slerp_Inter(&k1,&k2, 2*t*(1-t));
// }

// //主程序
// #define QUAD_SIZE 5    //插值顶点个数
// #define INSERT_POINTS_NUM 10    //每段Slerp插值点个数
// string squad_file = "../squad_points.txt";    //存储插值顶点坐标的四元数数值，目前为5个
// QUAT Qn[QUAD_SIZE], Sn[QUAD_SIZE], InsertPoints[(QUAD_SIZE-1)*(INSERT_POINTS_NUM-1) + QUAD_SIZE ];    //数组Qn,Sn,InsertPoints分别存储插值顶点、控制点和插值点坐标（个数为每段的插值点加插值顶点个数）

// int main(int argc, char** argv){
//     ifstream fin(squad_file );
//     if (!fin) {
//         cout << "cannot find squad_file at " << squad_file << endl;
//         return 1;
//     }
    
//     int qi=0;
//     while (!fin.eof()) {
//         double qx, qy, qz, qw;
//         fin >> qx >> qy >> qz >> qw;
//         Quat qn;
// 		qn.s = qw;
// 		qn.v[0] = qx;
// 		qn.v[1] = qy;
// 		qn.v[2] = qz;
//         Qn[qi++]=qn;
//     }
//     cout << "read total"<<sizeof(Qn)<<" quaternions entries"<<endl;

// 	//计算控制点
// 	GetCtlPoint(Qn, QUAD_SIZE, Sn);
// 	//计算插值点
// 	int ii=0;
// 	for (int i=0;i<QUAD_SIZE-1;i++){
//         InsertPoints[ii++]=Qn[i];
//         for (int t=0;t<1;t=t+1/INSERT_POINTS_NUM){
//             InsertPoints[ii++] = Squad(&Qn[i],&Sn[i],&Sn[i+1],&Qn[i+1],t);
//         }
//         InsertPoints[ii]=Qn[QUAD_SIZE-1];
//     }
// }
