#include "rrt.h"
#include <iostream>
using namespace std;

RRT::RRT()
{
    nmap.clear();
}
 void RRT::setStartAndGoal(int startx,int starty,int starttheta,int goalx,int goaly,int goaltheta)
 {

     startPos.x= startx;
     startPos.y = starty;
     startPos.z =starttheta;
     endPos.x = goalx;
     endPos.y = goaly;
     endPos.z =goaltheta;
     root = new Node;
     root->parent = NULL;
     root->position = startPos;
     root->cost=0;
     lastNode = root;
     nodes.push_back(root);
     goal=new Node;
     goal->position=endPos;
     goal->cost=0;
 }

void RRT::initialize()
{
    root = new Node;
    root->parent = NULL;
    root->position = startPos;
    root->cost=0;
    lastNode = root;
    nodes.push_back(root);
}
void RRT::getmapmatrix()
{
    ifstream f("/home/ljq/qt/rrtstar_rs/map.txt");
    string s;
    nmap.clear();
    while(getline(f,s))
    {
        string ss="";
        vector<int>temp;
        for(int i=0;i<s.size();i++)
        {
            if(s[i]==' ')
            {
                temp.push_back(atoi(ss.c_str()));
                ss="";
            }
            else
            {
                ss=ss+s[i];
            }
        }
        nmap.push_back(temp);
    }
    f.close();
}

Node* RRT::getRandomNode()
{
    Node* ret;
   XSVector3I point={rand() %500, rand() % 500,rand()%(360)-180};
    //XSVector3I point={rand() %300, rand() % 200,rand()%(360)-180};
    //Vector3i point(400,400,0);

        ret = new Node;
        ret->position = point;
        return ret;

    return NULL;
}

int RRT::distance(XSVector3I &p, XSVector3I &q)
{
    XSVector3I v ;
    v.x=p.x-q.x;
    v.y=p.y-q.y;
    v.z=p.z-q.z;
    return sqrt(powf(v.x, 2) + powf(v.y, 2)+powf(v.z/180*3.1415926,2));
}

Node* RRT::nearest(XSVector3I point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance(point, nodes[i]->position);
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

Node*RRT::newnearest(XSVector3I point)
{
    float minDist = 1e9;
    Node *closest = NULL;
    for(int i = 0; i < (int)nodes.size(); i++) {
        float dist = distance(point, nodes[i]->position)+nodes[i]->cost;
        if (dist < minDist) {
            minDist = dist;
            closest = nodes[i];
        }
    }
    return closest;
}

vector<vector<double> > RRT::outputRS(XSVector3I sssss,XSVector3I eeeeee)
{
        double q0[3]={double(sssss.x),double(sssss.y),double(sssss.z)/180*3.1415926};
        double q1[3]={double(eeeeee.x),double(eeeeee.y),double(eeeeee.z)/180*3.1415926};
         ReedsSheppStateSpace   *rrr=new ReedsSheppStateSpace;
         vector<vector<double > >finalpath;
             finalpath=rrr->xingshensample(q0,q1,0.5);
             return finalpath;
}

vector<vector<double> >  RRT::newConfig(Node *q, Node *qNearest)
{
    XSVector3I to = q->position;
   XSVector3I from = qNearest->position;
    //float theta=atan2(q->position.y()-qNearest->position.y(),q->position.x()-qNearest->position.x());
    vector<vector<double> >dddd;
    dddd=outputRS(from,to);
    return dddd;
}
float RRT::costt1(vector<vector<double> > in2)
{
    float ccccc=0;
    if(in2.size()>=1)
    {
        for(int i=0;i<in2.size()-1;i++)
        {
                    float tempdis=sqrt(pow(in2[i+1][0]-in2[i][0],2)+pow(in2[i+1][1]-in2[i][1],2));
                    ccccc=ccccc+tempdis;
        }
    }
    return ccccc;
}
float RRT::costt(vector<double > in,vector<int > in1)
{
    float c=0;
    for(int i=0;i<in.size();i++)
    {
        if(in[i]>=0)
        {
            c=c+abs(in[i]);
        }
        else
        {
             c=c+abs(in[i])*10;
        }
    }
    return c;
}
Node * RRT::rewire(Node *qnew)
{
    int indd=-2;
    for(int i=0;i<nodes.size();i++)
    {
        float dis=distance(nodes[i]->position,qnew->position);
        if(dis<20*6)
        {
            vector<vector<double> >dd;
            dd=outputRS(nodes[i]->position,qnew->position);
            float ddcost=costt1(dd);
            if( checkfeasi(dd)==0)
            {
                if(nodes[i]->cost+ddcost<qnew->cost)
                {
                     indd=i;
                    qnew->cost=nodes[i]->cost+ddcost;

                }
            }
        }
    }
if(indd!=-2)
{
    qnew->parent=nodes[indd];
    qnew->position.z=atan2(qnew->position.y-nodes[indd]->position.y,qnew->position.x-nodes[indd]->position.x);
}
    int indd1=-2;
    for(int i=1;i<nodes.size();i++)
    {
        if( nodes[i]!=nodes[indd])
        {
            float dis1=distance(nodes[i]->position,qnew->position);
            if(dis1<20*5)
            {
                vector<vector<double> >dd1;
                dd1=outputRS(nodes[i]->position,qnew->position);
                float ddcost1=costt1(dd1);
                if( checkfeasi(dd1)==0)
                {
                    if(qnew->cost+ddcost1<nodes[i]->cost)
                    {
                        indd1=i;
                        nodes[indd1]->cost=qnew->cost+ddcost1;
                        nodes[indd1]->parent=qnew;
                        qnew->position.z=atan2(qnew->position.y-nodes[indd1]->position.y,
                        qnew->position.x-nodes[indd1]->position.x);
                     }
                }
            }
        }
    }
    return qnew;
}
bool RRT::reached()
{
    if (distance(lastNode->position, endPos) <=200)
        return true;
    return false;
}

void RRT::setStepSize(int step)
{
    step_size = step;
}

void RRT::setMaxIterations(int iter)
{
    max_iter = iter;
}

int RRT::checkfeasi(vector<vector<double> > checkpoints)
{
    int finalflag=0;
    for (int i=0;i<checkpoints.size();i++)
    {
        XSVector2I pp={checkpoints[i][0],checkpoints[i][1]};
        double yaw=checkpoints[i][2];
        int chflag=0;
        int tempx=pp.x;
        int tempy=pp.y;
        if(  tempx <=1 || tempx >= 499 ||  tempy<= 1 ||  tempy >= 499 || int(nmap[tempx][tempy])==1)
        {
            chflag=1;
        }
         if(chflag==1)
        {
             finalflag=1;
             break;
         }
    }
    return finalflag;
}
