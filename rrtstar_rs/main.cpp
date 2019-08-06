#include <iostream>
#include "rrt.h"
#include <stdio.h>
#include <sys/time.h>
#include <stdlib.h>
#include <algorithm>
#define pi 3.1415926

using namespace std;

int main()
{
         struct timeval tv1;
         struct timeval tv2;
         float cbest=9999999;
         gettimeofday(&tv1,NULL);
         RRT* r=new RRT;
         r->getmapmatrix();
         int it=0;
         Node *newlast=new Node;
         r->setStartAndGoal(200,50,180,450,250,90);
         while(it<1000)
         {
             Node* q=r->getRandomNode();
             Node* qn=r->nearest(q->position);

             vector<vector<double > >outputrs;
             vector<int >RStypes;
             vector<double>RSlength;

             XSVector3I sssss0=qn->position;
             XSVector3I eeeeee0=q->position;
             double q0[3]={double(sssss0.x),double(sssss0.y),double(sssss0.z)/180*3.1415926};
             double q1[3]={double(eeeeee0.x),double(eeeeee0.y),double(eeeeee0.z)/180*3.1415926};
             ReedsSheppStateSpace   *rrr=new ReedsSheppStateSpace;
             outputrs=rrr->xingshensample(q0,q1,0.5);
             RStypes=rrr->xingshentype(q0,q1);
             for(int i=0;i<5;i++)
             {
                RSlength.push_back(rrr->reedsShepp(q0,q1).length_[0]);
             }
              int flag=0;
              int tttt=r->checkfeasi(outputrs);
              flag=tttt;
              if(flag==0)
              {
              float cc=r->costt(RSlength,RStypes);
              XSVector3I ret;
                  ret.x=outputrs[outputrs.size()-1][0];
                  ret.y=outputrs[outputrs.size()-1][1];
                  ret.z=outputrs[outputrs.size()-1][2]*180/pi;
                  if (ret.z>180)
                  {
                      while(ret.z>180)
                      {
                          ret.z=ret.z-360;
                      }
                  }
                  if (ret.z<-180)
                  {
                      while(ret.z<-180)
                      {
                          ret.z=ret.z+360;
                      }
                  }
                 Node* qnew=new Node;
                 qnew->position=ret;
                 qnew->parent=qn;
                 qnew->cost=qn->cost+cc;
                 Node * qnew1=new Node;
                 qnew1=r->rewire(qnew);
                 r->nodes.push_back(qnew1);
                 r->lastNode=qnew1;

            }
              if(r->reached())
              {
                  vector<vector<double> > outends;
                  outends=r->newConfig(r->lastNode,r->goal);
                  int finalch=0;
                 int finalcheck=r->checkfeasi(outends);
                 finalch=finalcheck;
                  if(finalch==0)
                  {
                      break;
                  }
              }
                it=it+1;
                cout<<"---------------------------::"<<it<<endl;
         }
        // cout<<"-------------------------------"<<endl;
        while ( r->lastNode!=NULL)
        {
            r->path.push_back( r->lastNode);
            r->lastNode= r->lastNode->parent;
        }

        reverse(r->path.begin(),r->path.end());
        Node * endpoint=new Node;
        endpoint->position=r->endPos;
        r->path.push_back( endpoint);

        vector<vector< vector<double> > >result;

        for(int i=0;i<r->path.size()-1;i++)
        {
            vector<vector<double> >result0;
            XSVector3I sssssnew={int(r->path[i]->position.x),int(r->path[i]->position.y),int(r->path[i]->position.z)};
            XSVector3I eeeenew={int(r->path[i+1]->position.x),int(r->path[i+1]->position.y),int(r->path[i+1]->position.z)};
            result0=r->outputRS(sssssnew,eeeenew);

            result.push_back(result0);

        }
	gettimeofday(&tv2,NULL);
 	float time_use=(tv2.tv_sec-tv1.tv_sec)*1000000+(tv2.tv_usec-tv1.tv_usec);
        vector<vector<double> >finalresult;
    for(int i=0;i<result.size();i++)
    {
        for(int j=0;j<result[i].size();j++)
        {
             vector<double>tt;
            tt.push_back(result[i][j][0]);
            tt.push_back(result[i][j][1]);
            tt.push_back(result[i][j][2]);
             finalresult.push_back(tt);
        }
    }
    for(int i=0;i<=finalresult.size()-2;i++)
    {
        int m=finalresult.size()-1;
        if(i>=m-1)
         {
            break;
        }
        int k=i+1;
         for(int j=i+2;j<=m;j++)
         {
             vector<vector<double> >result12;
             XSVector3I sssssnew={int(finalresult[i][0]),int(finalresult[i][1]),int(finalresult[i][2]*180/3.1415926)};
             XSVector3I eeeenew={int(finalresult[j][0]),int(finalresult[j][1]),int(finalresult[j][2]*180/3.1415926)};
            result12=r->outputRS(sssssnew,eeeenew);
            if(r->checkfeasi(result12)==0)
            {
                k=j;
            }
         }
         if(k>i+1)
         {
            finalresult.erase(finalresult.begin()+i+1,finalresult.begin()+k);
         }
    }
    vector<vector< vector<double> > >result1;
    vector<vector< vector<double> > >newresult;
    for(int i=0;i<finalresult.size()-1;i++)
    {
        vector<vector<double> >result13;

        XSVector3I sssssnew={int(finalresult[i][0]),int(finalresult[i][1]),int(finalresult[i][2]*180/3.1415926)};
        XSVector3I eeeenew={int(finalresult[i+1][0]),int(finalresult[i+1][1]),int(finalresult[i+1][2]*180/3.1415926)};

       result13=r->outputRS(sssssnew,eeeenew);
       newresult.push_back(result13);
    }
    result1=newresult;
            ofstream f("/home/ljq/qt/rrtstar_rs/getpath.txt",ios::out);
             for(int i=0;i<result1.size();i++)
               {
                        for(int j=0;j<result1[i].size();j++)
                        {
                           f<<result1[i][j][0]<<" "<<result1[i][j][1]<<" "<<result1[i][j][2]<<endl;

                        }
                    }
                    f<<r->endPos.x<<" "<<r->endPos.y<<" "<<double(r->endPos.z)/180*3.1415926<<endl;

                    f.close();

}
