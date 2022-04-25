// Triangle mesh viewer + corner table + subdivision + smoothing + compression + simplificatioin + geodesics + isolation
// Written by Jarek Rossignac June 2006. Modified February 2008
import processing.opengl.*;                // load OpenGL
String [] fn=  {"bunny.vts","horse.vts","torus.vts","tet.vts","fandisk.vts","squirrel.vts","venus.vts"};
int fni=0; int fniMax=fn.length;  

// ** SETUP **
void setup() { size(900, 900, P3D); setColors(); sphereDetail(6); //smooth();
  PFont font = loadFont("Courier-14.vlw"); textFont(font, 12);  // font for writing labels on screen
  M.declare(); M.makeGrid(10); M.init();
  initView(M);  
  img = loadImage("data/pic.jpg");
  } 
 
// ** DRAW **
void draw() {
  background(white); 
  perspective(PI/2.0,width/height,1.0,6.0*Rbox); 
  if (showHelpText) {camera(); translate(-290,-290,0); scale(1.7,1.7,1.0); showHelp(); showColors();  return; };
  lights(); directionalLight(0,0,128,0,1,0); directionalLight(0,0,128,0,0,1);
  translate(float(height)/2, float(height)/2, 0.0);     // center view wrt window  
  if ((!keyPressed)&&(mousePressed)) {C.pan(); C.pullE(); };
  if ((keyPressed)&&(mousePressed)) {updateView();}; 
  C1.track(C); C2.track(C1); C2.apply();  
  M.show();
  
  pt init_pt = M.g();
  vec init_dir = U(V(M.g(), M(M.g(M.n()), M.g(M.p()))));
  int init_corner = M.p();
  show_path(new Path(init_pt, init_dir, init_corner));
  
  if (currPos == null) currPos = init_pt;
  if (currPath == null) currPath = new Path(init_pt, init_dir, init_corner);
  if (showDiskRotation) diskRotation();
}

//***      KEY ACTIONS (details in keys tab)
void keyPressed() { keys(); };
void mousePressed() {C.anchor(); C.pose();   if (keyPressed&&(key=='m')) {C.setMark(); M.hitTriangle();}; };   // record where the cursor was when the mouse was pressed
void mouseReleased() {C.anchor(); C.pose(); };  // reset the view if any key was pressed when mouse was released 


void show_path(Path init_path)
{
  Path path = init_path;
  for(int i = 0; i < path_length; i++)
  {
    Path next_path = calculate_next_path(path);
    if(next_path == null)
    {
      return;
    }
    
    pt start = path.start;
    pt end = next_path.start;
    int c = path.start_edge_corner;
    pt A = M.g(M.n(c));
    pt B = M.g(M.p(c));
    pt C = M.g(c);
    
    draw_single_path(start, end);
    
    path = next_path;
  }
}

void draw_single_path(pt start, pt end)
{
  V(start, end).show(start);
}

Path calculate_next_path(Path curr_path)
{
    int c = curr_path.start_edge_corner;
    pt start = curr_path.start;
    vec dir = curr_path.dir;
    
    pt A = M.g(M.n(c));
    pt B = M.g(M.p(c));
    pt C = M.g(c);
    
    pt end1 = get_edge_intersection(start, dir, A, C);
    pt end2 = get_edge_intersection(start, dir, C, B);
    if(end1 != null && end2 != null && d(end1, end2) < 0.001f)
    {
       dir.sub(new vec(0.00001, 0.00001, 0.00001));
    }
    
    pt edge_point_A = A;
    pt edge_point_B = C;
    pt end = get_edge_intersection(start, dir, edge_point_A, edge_point_B);
    int next_c = M.o(M.p(c));
    if(end == null)
    {
      edge_point_A = C;
      edge_point_B = B;
      end = get_edge_intersection(start, dir, edge_point_A, edge_point_B);
      next_c = M.o(M.n(c));
    }
    if(end == null)
    {
      return null;
    }
    
    
    if(next_c == -1)
    {
       return null; 
    }
    
    vec org_vec_forward = U(cross(V(edge_point_A, edge_point_B), M.triNormal(M.t(c))));
    vec tan_vec = U(V(edge_point_A, edge_point_B));
    float s_forward = dot(dir, org_vec_forward);
    float s_tan = dot(dir, tan_vec);
    vec vec_forward = U(cross(V(edge_point_A, edge_point_B), M.triNormal(M.t(next_c))));
    dir = U(M(s_tan, tan_vec, s_forward, vec_forward));
    
    return new Path(end, dir, next_c);
}

pt get_edge_intersection(pt start, vec dir, pt edge_point_A, pt edge_point_B)
{
  vec N = cross(dir, cross(V(start, edge_point_A), V(start, edge_point_B)));
  
  float s = -1 * dot(V(start, edge_point_A), N) / dot(V(edge_point_A, edge_point_B), N);
  if(Float.isNaN(s))
  {
     return null; 
  }
  
  
  if(s < 0 || s > 1)
  {
     return null; 
  }
  
  return S(edge_point_A, s, edge_point_B);
}

class Path{
 pt start;
 vec dir;
 int start_edge_corner;
 
 Path(pt start, vec dir, int corner)
 {
    this.start = start;
    this.dir = dir;
    this.start_edge_corner = corner;
 }
}

// ========================= Part2 code =========================
boolean showDiskRotation;
int framesInAnimation;
float radius = 30;
float omega = TWO_PI/80;    // angular verlocity
pt currPos;

Path currPath;
Path nextPath;

boolean nextEdgeConvex;
boolean transitioning;
int transitionFrameCnt = 0;
pt diskPos;
pt currDiskPos;

int path_length = 30;
int curr_path_length = 0;
PImage img;

void resetForDiskRotation() {
  showDiskRotation = false;
  currPos = null;
  currPath = null;
  nextPath = null;
  transitioning = false;
  transitionFrameCnt = 0;
  curr_path_length = 0;
}

void updateNext(Path currPath) {
  System.out.println("updateNext() called");
  if (currPath == null) { 
    System.out.println("error currPath is null from updateNext()"); 
    return; 
  }
  
  nextPath = calculate_next_path(currPath);
  curr_path_length += 1;
  if (nextPath == null || curr_path_length >= path_length) {System.out.println("no more"); showDiskRotation = false; resetForDiskRotation(); return; }
  
  currPos = currPath.start;
  int c = currPath.start_edge_corner;
  pt A = M.g(M.n(c));
  pt B = M.g(M.p(c));
  pt C = M.g(c);
  
  int nc = nextPath.start_edge_corner;
  pt nC = M.g(nc);
  nextEdgeConvex = isConvexEdge(A, B, C, nC); 
}

void diskRotation() { 
  //System.out.println("diskRotation() is called");
  if (nextPath == null) { // executed once at the begining 
    transitioning = false;
    updateNext(currPath);
  }
  
  pt start = currPath.start;
  pt end = nextPath.start;
  vec dir = currPath.dir;
  
  int c = currPath.start_edge_corner;
  pt A = M.g(M.n(c));
  pt B = M.g(M.p(c));
  pt C = M.g(c);
  
  int nc = nextPath.start_edge_corner;
  pt nA = M.g(M.n(nc));
  pt nB = M.g(M.p(nc));
  pt nC = M.g(nc);
  
  
  if (transitioning) {
  //  transitionFrameCnt++;
  //  if (nextEdgeConvex) {
  //    vec currSurfaceN = M.triNormal(M.t(c));
  //    vec nextSurfaceN = M.triNormal(M.t(nc));
      
  //    float angle = angle(currSurfaceN, nextSurfaceN);
  //    framesInAnimation = max(1,(int) (angle/omega)) ;
  
  //    vec sn = L(currSurfaceN, nextSurfaceN, transitionFrameCnt/framesInAnimation);
  //    if (transitionFrameCnt < framesInAnimation) {
  //      fill(blue);
  //      pt diskO = moveAlongDir(currPos, radius, sn);
  //      disk(diskO, cross(sn, dir), radius);
  //      //showSelfRotation(diskO, cross(sn, dir), radius, true);
  //    } else {
  //      transitioning = false;
  //      transitionFrameCnt = 0;
  //      currPath = nextPath;
  //      updateNext(currPath);
  //    }
    
  //  } else {
  //    if (diskPos == null) {
  //      vec surfaceN = cross(V(A,B), V(A,C)); 
  //      diskPos = moveAlongDir(currPos, radius, surfaceN); 
  //      currDiskPos = diskPos;
  //      System.out.println("nextPath.start.z is " + nextPath.start.z);
  //      float distTotal = 2 * d(diskPos, nextPath.start); // forward and back
  //      float linearV = 2 * PI * radius * omega/10; // 5 is hard code
  //      framesInAnimation = (int)(distTotal/linearV);
  //      System.out.println("once only framesInAnimation = " + framesInAnimation);
  //    } 
  //    if (transitionFrameCnt < framesInAnimation) {
  //      float distance = d(diskPos, nextPath.start);
  //      float ratio = distance/20; //integral of sin(x) from 0 to PI is 2;
  //      float xx = ((float)transitionFrameCnt)/((float)framesInAnimation);
  //      float step = sin(TWO_PI * xx); // [-1, 1]
  //      currDiskPos = moveAlongDir(currDiskPos, step*ratio, V(diskPos, nextPath.start));
  //      System.out.println("currDiskPos.z = " + currDiskPos.z + " when transitionFrameCnt  = " + transitionFrameCnt);
        
  //      fill(blue);
  //      vec surfaceN = cross(V(A,B), V(A,C)); // cannot change order;
  //      disk(currDiskPos, cross(surfaceN, dir), radius);
  //      //drawDiskWithTecture(currDiskPos,cross(surfaceN, dir), 50);
  //      //showNoSelfRotation(currDiskPos, cross(surfaceN, dir), radius);
  //    } else {
  //      transitioning = false;
  //      transitionFrameCnt = 0;
  //      currPath = nextPath;
  //      updateNext(currPath);
  //    }
  //  }
  
      transitioning = false;
      transitionFrameCnt = 0;
      currPath = nextPath;
      updateNext(currPath);
      return;
  }
  
  // find newPos or set transitioning = true;
  float distance = 2 * PI * radius * omega/10;
  pt newPos = moveAlongDir(currPos, distance, dir);
  if (nextEdgeConvex && edgeIntersection(currPos, newPos, nA, nB)) {
    currPos = end;
    transitioning = true;
    System.out.println("convex transitioning = true");
  } else if (!nextEdgeConvex && distFromPtoNextPlane(newPos) < radius) {
    float angle = angle(M(dir), nextPath.dir);
    float dd = (float)(radius/Math.tan(angle/2));
    currPos = moveAlongDir(end, dd, M(dir));
    transitioning = true;
    
    System.out.println("concave transitioning = true");
  } else currPos = newPos;
 
  vec surfaceN = M.triNormal(M.t(c));
  pt diskO = moveAlongDir(currPos, radius, surfaceN);
  
  fill(blue);
  disk(diskO, cross(surfaceN, dir), radius);
  //drawDiskWithTecture(diskO, cross(surfaceN, dir), 50);
  //showSelfRotation(diskO, cross(surfaceN, dir), radius, false);
}

// ***************** display *****************
float angleCnt = 0;
void showSelfRotation(pt P, vec V, float r, boolean transition) {
  vec I = U(Normal(V));
  vec J = U(N(I,V));
  if (!transition) angleCnt++;
  else angleCnt += 0.2;
  float a = -(angleCnt *omega) % TWO_PI; // 20 is hard code
  
  push();
  fill(magenta);
  show(P(P,r/2*cos(a),I,r/2*sin(a),J), 10);
  pop();
}
void showNoSelfRotation(pt P, vec V, float r) {
  vec I = U(Normal(V));
  vec J = U(N(I,V));
  float a = -(angleCnt *omega) % TWO_PI; // 20 is hard code
  
  push();
  fill(magenta);
  show(P(P,r/2*cos(a),I,r/2*sin(a),J), 10);
  pop();
}

// ********* helper methods *********
boolean edgeIntersection(pt A, pt B, pt C, pt D) { //given co-planar
  return ( 0 >= dot(cross(V(A, B), V(A, C)), cross(V(A, B), V(A, D))) &&
  0 >= dot(cross(V(C, D), V(C, A)), cross(V(C, D), V(C, B))) );
}

pt moveAlongDir(pt from, float dist, vec dir) {
  return P(from, U(dir).scaleBy(dist));
}

boolean isConvexEdge(pt A, pt B, pt C, pt next) { //  ABC is tri vertices in cw 
  return volume(A, B, C, next) <= 0;
}

float distFromPtoNextPlane(pt pos) {
  vec PO = V(pos, nextPath.start);
  vec n = U(nextPath.dir);
  vec ref = M(PO, V(dot(PO, n) * 2, n));
  pt refPos = P(P(pos, PO), ref);
  return d(pos, refPos)/2;
}

float multiply(pt P, vec V) {
  return (P.x * V.x + P.y * V.y + P.z * V.z);
}

vec L(vec A, vec B, float t) {return new vec(A.x+t*(B.x-A.x),A.y+t*(B.y-A.y),A.z+t*(B.z-A.z));}

void disk(pt P, vec V, float r) {  
  vec I = U(Normal(V));
  vec J = U(N(I,V));
  disk(P,I,J,r);
  }
  
float da = TWO_PI/32;
float sa = 0;
void disk(pt P, vec I, vec J, float r) {
  noStroke();
  textureMode(NORMAL);
  fill(white);
  beginShape(TRIANGLE_FAN);
  texture(img);
  
    vertex(P.x,P.y,P.z, 0.5, 0.5);
    for(float a=0; a<=TWO_PI+da; a+=da)
    {
      pt p = P(P,r*cos(a),I,r*sin(a),J);
      vertex(p.x,p.y,p.z, 0.5 + cos(sa + a) / 2.0, 0.5 + sin(sa + a) / 2.0);
    }
    sa = (sa + omega) % TWO_PI;
    
  endShape();
  }
