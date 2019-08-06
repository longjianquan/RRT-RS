#ifndef RRT_H
#define RRT_H
#include <stdlib.h>
#include <vector>
#include <math.h>
#include "rs.h"
#include <fstream>

using namespace std;

struct XSVector3I
{
    int x;
    int y;
    int z;
};
struct XSVector2I
{
    int x;
    int y;
};
struct Node {
    Node *parent;
    float cost;
    XSVector3I position;
};

class RRT
{
public:
    RRT();
    void setStartAndGoal(int startx,int starty,int starttheta,int goalx,int goaly,int goaltheta);
    void initialize();
    vector<vector<double> > outputRS(XSVector3I sssss,XSVector3I eeeeee);
    ReedsSheppStateSpace outputRS1(Node *q, Node *qNearest);
    void getmapmatrix();
    Node* getRandomNode();
    Node* nearest(XSVector3I point);
    Node* newnearest(XSVector3I point);
    Node* rewire(Node *qnew);
    int distance(XSVector3I &p, XSVector3I &q);
    vector<vector<double> > newConfig(Node *q, Node *qNearest);
    int checkfeasi(vector<vector<double> > checkpoints);
    bool reached();
    void setStepSize(int step);

    void setMaxIterations(int iter);
    float costt(vector<double > in,vector<int > in1);
    float costt1(vector<vector<double> > in2);
    vector<vector<int> >nmap;
    vector<Node *> nodes;
    vector<Node *> nodes2;
    vector<Node *> path;
    Node *root, *lastNode,*goal;
    XSVector3I startPos, endPos;
    int max_iter;
    int step_size;
};
#endif // RRT_H
