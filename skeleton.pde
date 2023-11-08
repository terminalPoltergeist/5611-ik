// started from ik in-class exercise

void setup(){
  size(640,480);
}

Vec2 ball = new Vec2(200,400);
Vec2 body = new Vec2(320,240);
Vec2 obstacle = new Vec2(500,360);

boolean click, lgrabbed, rgrabbed, lrgrabbed, rrgrabbed;

void mousePressed() {
  click = true;
}

void mouseReleased() {
click = false;
}

//{{{ right
//root
Vec2 rroot = new Vec2(380,240);
//upper arm
float rl0 = 100; 
float ra0 = 0.3; //shoulder joint
//lower arm
float rl1 = 80;
float ra1 = 0.3; //elbow joint
//hand
float rl2 = 20;
float ra2 = 0.3; //wrist joint

Vec2 start_rl1,start_rl2,rendPoint;
//}}}

//{{{ left
//root
Vec2 lroot = new Vec2(260,240);
//upper arm
float ll0 = 100; 
float la0 = 0.3 - (3.14159/2); //shoulder joint
//lower arm
float ll1 = 80;
float la1 = 0.3 - (3.14159/2); //elbow joint
//hand
float ll2 = 20;
float la2 = 0.3 - (3.14159/2); //wrist joint

Vec2 start_ll1,start_ll2,lendPoint;
//}}}

// {{{ right funcs
void rsolve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update wrist joint
  startToGoal = goal.minus(start_rl2);
  startToEndEffector = rendPoint.minus(start_rl2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, 0.1);
  if (cross(startToGoal,startToEndEffector) < 0)
    ra2 += angleDiff;
  else
    ra2 -= angleDiff;
  ra2 = (ra2/abs(ra2)) * min(abs(ra2)*0.9, 3.1415926/2*0.9);
  rfk(); //Update link positions with fk (e.g. end effector changed)

    //Update elbow joint
  startToGoal = goal.minus(start_rl1);
  startToEndEffector = rendPoint.minus(start_rl1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, 0.05);
  if (cross(startToGoal,startToEndEffector) < 0)
    ra1 += angleDiff;
  else
    ra1 -= angleDiff;
  ra1 = min(-4, max(ra1, -9));
  rfk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update shoulder joint
  startToGoal = goal.minus(rroot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = rendPoint.minus(rroot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, 0.03);
  if (cross(startToGoal,startToEndEffector) < 0)
    ra0 += angleDiff;
  else
    ra0 -= angleDiff;
  // cap angle at 90deg (pi/2 rad) and floor it at 0
  ra0 = max(0, min(ra0, 3.1415926/2*0.9));
  rfk(); //Update link positions with fk (e.g. end effector changed)
 
  //println("Angle 0:",ra0,"Angle 1:",ra1,"Angle 2:",ra2);

}

void rfk(){
  start_rl1 = new Vec2(cos(ra0)*rl0,sin(ra0)*rl0).plus(rroot);
  start_rl2 = new Vec2(cos(ra0+ra1)*rl1,sin(ra0+ra1)*rl1).plus(start_rl1);
  rendPoint = new Vec2(cos(ra0+ra1+ra2)*rl2,sin(ra0+ra1+ra2)*rl2).plus(start_rl2);
}
// }}}

// {{{ left funcs
void lsolve(){
  Vec2 goal = new Vec2(mouseX, mouseY);
  
  Vec2 startToGoal, startToEndEffector;
  float dotProd, angleDiff;
  
  //Update wrist joint
  startToGoal = goal.minus(start_ll2);
  startToEndEffector = lendPoint.minus(start_ll2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, angleDiff);
  if (cross(startToGoal,startToEndEffector) < 0)
    la2 += angleDiff;
  else
    la2 -= angleDiff;
  //la2 = (la2/abs(la2)) * min(abs(la2)*0.9, 3.1415926/2*0.9);
  lfk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update elbow joint
  startToGoal = goal.minus(start_ll1);
  startToEndEffector = lendPoint.minus(start_ll1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, angleDiff);
  if (cross(startToGoal,startToEndEffector) < 0)
    la1 += angleDiff;
  else
    la1 -= angleDiff;
  //la1 = max(4, min(la1, 9));
  lfk(); //Update link positions with fk (e.g. end effector changed)
  
  //Update shoulder joint
  startToGoal = goal.minus(lroot);
  if (startToGoal.length() < .0001) return;
  startToEndEffector = lendPoint.minus(lroot);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  angleDiff = acos(dotProd);
  angleDiff = min(angleDiff, angleDiff);
  if (cross(startToGoal,startToEndEffector) < 0)
    la0 += angleDiff;
  else
    la0 -= angleDiff;
  //la0 = min(3.14159, max(la0, 3.14159/2/0.9));
  // cap angle at 90deg (pi/2 lad) and floor it at 0
  lfk(); //Update link positions with fk (e.g. end effector changed)
 
  //println("Angle 0:",la0,"Angle 1:",la1,"Angle 2:",la2);
}

void lfk(){
  start_ll1 = new Vec2(cos(la0)*ll0,sin(la0)*ll0).plus(lroot);
  start_ll2 = new Vec2(cos(la0+la1)*ll1,sin(la0+la1)*ll1).plus(start_ll1);
  lendPoint = new Vec2(cos(la0+la1+la2)*ll2,sin(la0+la1+la2)*ll2).plus(start_ll2);
}
//}}}

// {{{ draw
float armW = 20;
void draw(){
  rfk();
  rsolve();
  lfk();
  lsolve();

  background(250,250,250);

  float ldist = ball.minus(lendPoint).length();
  float rdist = ball.minus(rendPoint).length();
  Vec2 m = new Vec2(mouseX, mouseY);
  float lrdist = m.minus(lroot).length();
  float rrdist = m.minus(rroot).length();
  
  if (click) {
    if (ldist < 10) {
      lgrabbed = true;
    }
    if (lgrabbed && ldist < rdist) {
      ball.x = lendPoint.x;
      ball.y = lendPoint.y;
    }
    if (rdist < 10) {
      rgrabbed = true;
    }
    if (rgrabbed && rdist < ldist) {
      ball.x = rendPoint.x;
      ball.y = rendPoint.y;
    }

    if (lrdist < 10) {
      lrgrabbed = true;
    }
    if (lrgrabbed) {
      lroot = new Vec2(m.x, m.y);
    }
    if (rrdist < 10) {
      rrgrabbed = true;
    }
    if (rrgrabbed) {
      rroot = new Vec2(m.x, m.y);
    }
  }
  if (!click) {
    lgrabbed = false;
    rgrabbed = false;
    lrgrabbed = false;
    rrgrabbed = false;
  }
  
  fill(178, 187, 183);

  pushMatrix();
  translate(body.x, body.y);
  rect(-60, -120, 120, 240, 5);
  popMatrix();

  fill(0,0,255);

  circle(ball.x, ball.y,40);
  circle(obstacle.x, obstacle.y, 50);

  fill(178, 187, 183);

  pushMatrix();
  translate(rroot.x,rroot.y);
  rotate(ra0);
  rect(0, -armW/2, rl0, armW, 5);
  popMatrix();
  
  pushMatrix();
  translate(lroot.x,lroot.y);
  rotate(la0);
  rect(0, -armW/2, ll0, armW, 5);
  popMatrix();

  pushMatrix();
  translate(start_rl1.x,start_rl1.y);
  rotate(ra0+ra1);
  rect(0, -armW/2*0.9, rl1, armW*0.9, 5);
  popMatrix();
  
  pushMatrix();
  translate(start_ll1.x,start_ll1.y);
  rotate(la0+la1);
  rect(0, -armW/2*0.9, ll1, armW*0.9, 5);
  popMatrix();

  pushMatrix();
  translate(start_rl2.x,start_rl2.y);
  rotate(ra0+ra1+ra2);
  rect(0, -armW/2, rl2, armW, 7);
  if (!click){
    rect(armW/1.5, armW/4, rl2/1.75, armW/4, 10);
    rect(armW/1.5, 0, rl2/1.5, armW/4, 10);
    rect(armW/1.5, -armW/4, rl2/1.5, armW/4, 10);
    rect(armW/1.5, -armW/2, rl2/1.75, armW/4, 10);
  }
  rect(armW/2.5, armW/2, rl2/2.25, armW/2, 10);
  popMatrix();
  
  pushMatrix();
  translate(start_ll2.x,start_ll2.y);
  rotate(la0+la1+la2);
  rect(0, -armW/2, ll2, armW, 7);
  rect(armW/2.5, -armW/2, ll2/2.25, -armW/2, 10);
  if (!click) {
    rect(armW/1.5, armW/4, ll2/1.75, armW/4, 10);
    rect(armW/1.5, 0, ll2/1.5, armW/4, 10);
    rect(armW/1.5, -armW/4, ll2/1.5, armW/4, 10);
    rect(armW/1.5, -armW/2, ll2/1.75, armW/4, 10);
  }
  popMatrix();
}
  
// {{{ vec2
//-----------------
// Vector Library
//-----------------

//Vector Library
//CSCI 5611 Vector 2 Library [Example]
// Stephen J. Guy <sjguy@umn.edu>

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
// }}}
