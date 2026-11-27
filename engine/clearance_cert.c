/* Conservative mesh separation for rigid translations. Build with tools/build_clearance_wasm.py.
 * Every early-out is a separating projection or an unobstructed ray to infinity.
 * Ambiguous, touching, malformed and contained pairs go to the ordinary witness query. */
#include <stdint.h>
#define fabs __builtin_fabs
#define sqrt __builtin_sqrt
#define isfinite __builtin_isfinite
#define INFINITY __builtin_inf()

typedef struct { double x,y,z; } Vec;
typedef struct { Vec lo,hi; } Box;
typedef struct { double lo1,hi1,lo2,hi2; } Diagonals;
typedef struct { Box box; Diagonals diag; int left,right,start,count; } Node;
typedef struct { int i,j,k; } Tri;
typedef struct {
  Vec *p; Tri *tri; int *order,*parent,*seed; Node *nodes; Box *boxes,*components; Diagonals *diag;
  int vertices,faces,nodesUsed,seeds; double scale;
  double nx,nz; int cached; struct {double nx,nz,lo,hi;} plane[12];
} Mesh;
typedef struct { Mesh *a,*b; Vec t,travel,loTravel,hiTravel; double threshold,minimum; Diagonals move; } Query;
static double mn(double a,double b){return a<b?a:b;}
static double mx(double a,double b){return a>b?a:b;}
static Vec sub(Vec a,Vec b){return (Vec){a.x-b.x,a.y-b.y,a.z-b.z};}
static Vec add(Vec a,Vec b){return (Vec){a.x+b.x,a.y+b.y,a.z+b.z};}
static Vec neg(Vec a){return (Vec){-a.x,-a.y,-a.z};}
static double dot(Vec a,Vec b){return a.x*b.x+a.y*b.y+a.z*b.z;}
static Vec cross(Vec a,Vec b){return (Vec){a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
static double at(Vec p,int k){return k==0?p.x:k==1?p.y:p.z;}
static double mag(Vec p){return mx(fabs(p.x),mx(fabs(p.y),fabs(p.z)));}
static Box empty(void){return (Box){{INFINITY,INFINITY,INFINITY},{-INFINITY,-INFINITY,-INFINITY}};}
static void include(Box *b,Vec p){
  b->lo=(Vec){mn(b->lo.x,p.x),mn(b->lo.y,p.y),mn(b->lo.z,p.z)};
  b->hi=(Vec){mx(b->hi.x,p.x),mx(b->hi.y,p.y),mx(b->hi.z,p.z)};
}
static int root(int *p,int i){while(p[i]!=i){p[i]=p[p[i]];i=p[i];}return i;}
// Two additional signed unit directions in the working X/Z section.
static Diagonals projected(Vec p){
  double u=(p.x+p.z)*0.7071067811865475,v=(p.x-p.z)*0.7071067811865475;
  return (Diagonals){u,u,v,v};
}
static Diagonals merge_diag(Diagonals a,Diagonals b){
  return (Diagonals){mn(a.lo1,b.lo1),mx(a.hi1,b.hi1),mn(a.lo2,b.lo2),mx(a.hi2,b.hi2)};
}
static double diag_gap(Diagonals a,Diagonals b,Diagonals d){
  return mx(mx(a.lo1+d.lo1-b.hi1,b.lo1-(a.hi1+d.hi1)),mx(a.lo2+d.lo2-b.hi2,b.lo2-(a.hi2+d.hi2)));
}
static int branch(Mesh *m,int start,int count){
  int id=m->nodesUsed++;Node *node=&m->nodes[id];Box box=empty();
  for(int k=start;k<start+count;k++){Box f=m->boxes[m->order[k]];include(&box,f.lo);include(&box,f.hi);}
  Diagonals diag=m->diag[m->order[start]];
  for(int k=start+1;k<start+count;k++)diag=merge_diag(diag,m->diag[m->order[k]]);
  *node=(Node){box,diag,-1,-1,start,count};if(count<=5)return id;
  Vec span=sub(box.hi,box.lo);int axis=span.x>=span.y&&span.x>=span.z?0:span.y>=span.z?1:2;
  double pivot=(at(box.lo,axis)+at(box.hi,axis))*1.5;
  int mid=start;
  for(int k=start;k<start+count;k++){
    Tri f=m->tri[m->order[k]];
    if(at(m->p[f.i],axis)+at(m->p[f.j],axis)+at(m->p[f.k],axis)<pivot){
      int tmp=m->order[k];m->order[k]=m->order[mid];m->order[mid++]=tmp;
    }
  }
  if(mid-start<count/8+1||start+count-mid<count/8+1)mid=start+count/2;
  node->left=branch(m,start,mid-start);node->right=branch(m,mid,start+count-mid);return id;
}
int mesh_bytes(int n,int f){return sizeof(Mesh)+(sizeof(Vec)+sizeof(Box))*n+sizeof(Tri)*f+sizeof(int)*(f+2*n)+8+(sizeof(Node)*2+sizeof(Box)+sizeof(Diagonals))*f;}
int vertex_offset(void){return sizeof(Mesh);}
int prepare(Mesh *m,int n,int f){
  m->p=(Vec*)(m+1);m->tri=(Tri*)(m->p+n);m->order=(int*)(m->tri+f);
  m->parent=m->order+f;m->seed=m->parent+n;
  m->nodes=(Node*)(((uintptr_t)(m->seed+n)+7u)&~7u);m->vertices=n;m->faces=f;m->nodesUsed=0;m->seeds=0;m->scale=1;
  m->boxes=(Box*)(m->nodes+2*f);m->components=m->boxes+f;m->diag=(Diagonals*)(m->components+n);
  for(int i=0;i<n;i++){
    Vec p=m->p[i];if(!isfinite(p.x)||!isfinite(p.y)||!isfinite(p.z)||mag(p)>1e7)return 0;
    m->scale=mx(m->scale,mag(p));m->parent[i]=i;
  }
  double longest=0;m->nx=0;m->nz=0;m->cached=0;
  for(int i=0;i<f;i++){
    Tri t=m->tri[i];if(t.i<0||t.i>=n||t.j<0||t.j>=n||t.k<0||t.k>=n)return 0;
    int r=root(m->parent,t.i);m->parent[root(m->parent,t.j)]=r;m->parent[root(m->parent,t.k)]=r;
    m->order[i]=i;
    Vec xyz[]={m->p[t.i],m->p[t.j],m->p[t.k]};
    for(int j=0;j<3;j++){
      Vec edge=sub(xyz[(j+1)%3],xyz[j]);double len=edge.x*edge.x+edge.z*edge.z;
      if(len>longest){longest=len;m->nx=-edge.z;m->nz=edge.x;}
    }
    Box box=empty();include(&box,m->p[t.i]);include(&box,m->p[t.j]);include(&box,m->p[t.k]);m->boxes[i]=box;
    m->diag[i]=merge_diag(projected(m->p[t.i]),merge_diag(projected(m->p[t.j]),projected(m->p[t.k])));
  }
  // Shared indices imply connectivity. Unwelded coincident vertices merely
  // increase the number of components that require outside proofs.
  for(int i=0;i<n;i++)m->components[i]=empty();
  for(int i=0;i<f;i++){
    Tri t=m->tri[i];int r=root(m->parent,t.i);Box *bounds=&m->components[r];
    if(bounds->lo.x==INFINITY)m->seed[m->seeds++]=r;
    include(bounds,m->p[t.i]);include(bounds,m->p[t.j]);include(bounds,m->p[t.k]);
  }
  if(longest>0){double length=sqrt(longest);m->nx/=length;m->nz/=length;}
  branch(m,0,f);return 1;
}
static double axis_gap(Vec n,Vec a,Vec b,Vec c,Vec d,Vec e,Vec f,Vec travel){
  double l2=dot(n,n);if(!(l2>1e-34))return 0;
  double u=dot(n,a),v=dot(n,b),w=dot(n,c),x=dot(n,d),y=dot(n,e),z=dot(n,f);
  double sweep=dot(travel,n);
  double gap=mx(mn(u,mn(v,w))+mn(0,sweep)-mx(x,mx(y,z)),mn(x,mn(y,z))-mx(u,mx(v,w))-mx(0,sweep));
  return gap>0?gap/sqrt(l2):0;
}
static int test_axis(Query*q,Vec n,Vec a,Vec b,Vec c,Vec d,Vec e,Vec f){
  double g=axis_gap(n,a,b,c,d,e,f,q->travel);
  if(g<=q->threshold)return 0;
  q->minimum=mn(q->minimum,g);return 1;
}
static int apart(Query*q,int ia,int ib){
  Tri ta=q->a->tri[ia],tb=q->b->tri[ib];Vec a=add(q->a->p[ta.i],q->t),b=add(q->a->p[ta.j],q->t),c=add(q->a->p[ta.k],q->t);
  Vec d=q->b->p[tb.i],e=q->b->p[tb.j],f=q->b->p[tb.k];
  Vec axesA[]={sub(b,a),sub(c,b),sub(a,c)},axesB[]={sub(e,d),sub(f,e),sub(d,f)};
  Vec na=cross(axesA[0],axesA[1]),nb=cross(axesB[0],axesB[1]);
  if(test_axis(q,na,a,b,c,d,e,f)||test_axis(q,nb,a,b,c,d,e,f))return 1;
  for(int i=0;i<3;i++){
    if(test_axis(q,cross(na,axesA[i]),a,b,c,d,e,f)||test_axis(q,cross(nb,axesB[i]),a,b,c,d,e,f))return 1;
    for(int j=0;j<3;j++)if(test_axis(q,cross(axesA[i],axesB[j]),a,b,c,d,e,f))return 1;
  }
  return 0;
}
static double boxgap(Box a, Box b,Query*q){
  Vec lo=q->loTravel,hi=q->hiTravel;
  double x=mx(a.lo.x+lo.x-b.hi.x,b.lo.x-(a.hi.x+hi.x));
  double y=mx(a.lo.y+lo.y-b.hi.y,b.lo.y-(a.hi.y+hi.y));
  double z=mx(a.lo.z+lo.z-b.hi.z,b.lo.z-(a.hi.z+hi.z));
  return mx(x,mx(y,z));
}
static int separated_nodes(Query*q,int ia,int ib){
  Node *a=&q->a->nodes[ia],*b=&q->b->nodes[ib];
  double distance=boxgap(a->box,b->box,q);
  if(distance<=q->threshold)distance=diag_gap(a->diag,b->diag,q->move);
  if(distance>q->threshold){q->minimum=mn(q->minimum,distance);return 1;}
  if(a->left<0&&b->left<0){
    for(int i=a->start;i<a->start+a->count;i++)for(int j=b->start;j<b->start+b->count;j++){
      int ia=q->a->order[i],ib=q->b->order[j];
      double g=boxgap(q->a->boxes[ia],q->b->boxes[ib],q);
      if(g<=q->threshold)g=diag_gap(q->a->diag[ia],q->b->diag[ib],q->move);
      if(g>q->threshold){q->minimum=mn(q->minimum,g);continue;}
      if(!apart(q,ia,ib))return 0;
    }
    return 1;
  }
  if(b->left>=0&&(a->left<0||b->count>a->count))return separated_nodes(q,ia,b->left)&&separated_nodes(q,ia,b->right);
  return separated_nodes(q,a->left,ib)&&separated_nodes(q,a->right,ib);
}
static int outside_box(Vec p,Box b,double eps){
  return p.x<b.lo.x-eps||p.x>b.hi.x+eps||p.y<b.lo.y-eps||p.y>b.hi.y+eps||p.z<b.lo.z-eps||p.z>b.hi.z+eps;
}
// A clear axis ray to infinity is sufficient even for a concave shell.
// Node boxes alone may block a ray unnecessarily; that invokes the exact solver.
static int clear_ray(Mesh*m,int node,Vec p,int direction,double eps){
  Node*n=&m->nodes[node];int axis=direction/2;
  for(int k=0;k<3;k++){
    double v=at(p,k),lo=at(n->box.lo,k)-eps,hi=at(n->box.hi,k)+eps;
    if(k!=axis?(v<lo||v>hi):((direction&1)?v<lo:v>hi))return 1;
  }
  if(n->left<0){
    int u=(axis+1)%3,v=(axis+2)%3;
    for(int i=n->start;i<n->start+n->count;i++){
      int index=m->order[i];Box box=m->boxes[index];int miss=0;
      for(int k=0;k<3;k++){
        double w=at(p,k),lo=at(box.lo,k)-eps,hi=at(box.hi,k)+eps;
        if(k!=axis?(w<lo||w>hi):((direction&1)?w<lo:w>hi)){miss=1;break;}
      }
      if(miss)continue;
      Tri f=m->tri[index];Vec q[]={m->p[f.i],m->p[f.j],m->p[f.k]};
      for(int j=0;j<3;j++){
        Vec a=q[j],edge=sub(q[(j+1)%3],a),op=sub(q[(j+2)%3],a),pos=sub(p,a);
        double dx=at(edge,u),dy=at(edge,v),limit=eps*(1+fabs(dx)+fabs(dy));
        double side=dx*at(op,v)-dy*at(op,u),test=dx*at(pos,v)-dy*at(pos,u);
        if((side>limit&&test< -limit)||(side< -limit&&test>limit)){miss=1;break;}
      }
      if(!miss)return 0;
    }
    return 1;
  }
  return clear_ray(m,n->left,p,direction,eps)&&clear_ray(m,n->right,p,direction,eps);
}
static int outside(Mesh*a,Mesh*b,Vec t,double eps){
  for(int i=0;i<a->seeds;i++){
    int root=a->seed[i];Box c=a->components[root],other=b->nodes[0].box;
    // With the surfaces already separated, a connected component cannot be
    // partly inside a closed shell. Any extremal vertex outside the opposing
    // box proves the entire component is outside.
    if(outside_box(add(c.lo,t),other,eps)||outside_box(add(c.hi,t),other,eps))continue;
    Vec p=add(a->p[root],t);
    int found=0;
    for(int k=0;k<6&&!found;k++)found=clear_ray(b,0,p,k,eps);
    if(!found)return 0;
  }
  return 1;
}
// An arbitrary direction is sufficient for a separating projection. The
// normal to the longest projected edge usually follows a flat sheet's skin.
static void interval(Mesh *m,double nx,double nz,double *lo,double *hi){
  for(int i=0;i<m->cached;i++)if(m->plane[i].nx==nx&&m->plane[i].nz==nz){
    *lo=m->plane[i].lo;*hi=m->plane[i].hi;return;
  }
  *lo=INFINITY;*hi=-INFINITY;
  for(int i=0;i<m->vertices;i++){
    double value=nx*m->p[i].x+nz*m->p[i].z;*lo=mn(*lo,value);*hi=mx(*hi,value);
  }
  if(m->cached<12){
    int i=m->cached++;m->plane[i].nx=nx;m->plane[i].nz=nz;m->plane[i].lo=*lo;m->plane[i].hi=*hi;
  }
}
static double sheet_gap(Mesh*a,Mesh*b,Vec t,Vec d,double nx,double nz){
  double al,ah,bl,bh;interval(a,nx,nz,&al,&ah);interval(b,nx,nz,&bl,&bh);
  double shift=t.x*nx+t.z*nz,delta=d.x*nx+d.z*nz;
  return mx(al+shift+mn(0,delta)-bh,bl-(ah+shift+mx(0,delta)));
}
double separation(Mesh*a,Mesh*b,double x,double y,double z,double margin,double dx,double dy,double dz){
  Vec t={x,y,z},travel={dx,dy,dz};double eps=1e-10*(1+a->scale+b->scale+mag(t)+mag(travel));
  Vec lo={x+mn(0,dx),y+mn(0,dy),z+mn(0,dz)},hi={x+mx(0,dx),y+mx(0,dy),z+mx(0,dz)};
  Query q={a,b,t,travel,lo,hi,margin+eps,INFINITY,merge_diag(projected(t),projected(add(t,travel)))};
  double gap=sheet_gap(a,b,t,travel,a->nx,a->nz);
  if(gap>q.threshold)return gap-eps;
  gap=sheet_gap(a,b,t,travel,b->nx,b->nz);
  if(gap>q.threshold)return gap-eps;
  if(!separated_nodes(&q,0,0))return -1;
  if(!outside(a,b,t,eps))return -2;
  if(!outside(b,a,neg(t),eps))return -3;
  return q.minimum-eps;
}

// Scan projected sheet edges against the lower punch profile in the same
// order as the reference evaluator. Keep the first witness on ties.
// Spatial rejection in original edge order preserves the exact winning
// triangle and profile edge, even when several faces give identical heights.
#define EDGE_BLOCK 16
void tip_blocks(const double *edges,int count,double *ranges){
  for(int i=0;i<count;i+=EDGE_BLOCK){
    double lo=INFINITY,hi=-INFINITY;
    for(int k=i;k<count&&k<i+EDGE_BLOCK;k++){
      lo=mn(lo,mn(edges[4*k],edges[4*k+2]));hi=mx(hi,mx(edges[4*k],edges[4*k+2]));
    }
    ranges[2*(i/EDGE_BLOCK)]=lo;ranges[2*(i/EDGE_BLOCK)+1]=hi;
  }
}
int required_tip(const double *edges,int count,const double *ranges,const double *profile,int faces,
                 double x,double z,double maximum,double *result){
  int found=0;
  double minX=INFINITY,maxX=-INFINITY;
  for(int j=0;j<faces;j++){minX=mn(minX,profile[6*j]);maxX=mx(maxX,profile[6*j+1]);}
  for(int i=0;i<count;i++){
    if(i%EDGE_BLOCK==0){
      const double *range=ranges+2*(i/EDGE_BLOCK);
      if(range[1]+x<minX-1e-9||maxX<range[0]+x-1e-9){i+=EDGE_BLOCK-1;continue;}
    }
    const double *e=edges+4*i;
    double ax=e[0]+x,az=e[1]+z,bx=e[2]+x,bz=e[3]+z,dx=bx-ax,dz=bz-az;
    double left=mn(ax,bx),right=mx(ax,bx);
    if(right<minX-1e-9||maxX<left-1e-9)continue;
    for(int j=0;j<faces;j++){
      const double *s=profile+6*j;
      double lo=mx(left,s[0]),hi=mn(right,s[1]);
      if(hi<lo-1e-9)continue;
      for(int boundary=0;boundary<(lo==hi?1:2);boundary++){
        double xx=boundary?hi:lo;
        double part=fabs(dx)<1e-9?mx(az,bz):az+dz*((xx-ax)/dx);
        double tool=s[5]?s[4]:s[2]*xx+s[3],height=part-tool;
        if(height>maximum){
          maximum=height;result[0]=height;result[1]=i;result[2]=j;result[3]=xx;found=1;
        }
      }
    }
  }
  return found;
}
