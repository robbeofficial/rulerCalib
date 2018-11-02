import java.util.*;

Node n[] = {
  new Node(100, 100, 0, 0), 
  new Node(200, 100, 1000, 0), 
  new Node(200, 200, 1000, 1000), 
  new Node(100, 200, 0, 1000), 
};

Edge e[] = {
  new Edge(n[0], n[1], 100), 
  new Edge(n[1], n[2], 100), 
  new Edge(n[2], n[3], 100), 
  new Edge(n[3], n[0], 100), 
  new Edge(n[0], n[2], 100), 
  //new Edge(n[1], n[3], 100),
};

enum Mode {
  ROUGH, FINE, DONE
}

Mode mode = Mode.DONE;
float pMillis = millis();

Node dragged = null;
int currentEdge = -1;
String currentLength = "";

float fps = 0;
double[][] H, Hinv;

double[] center;

PVector mouse = new PVector(mouseX, mouseY);
PVector dragStart; 
boolean mouseDragged;

LinkedList<String> logs = new LinkedList();
LinkedList<Integer> logMillis = new LinkedList();

PImage img;

void setup() {
  fullScreen(P3D);
  //size(640, 480, P3D);
  smooth(8);
  //load();
  noCursor();

  img = loadImage("export2_inv.png");

  //frameRate(60);  
  //ffmpegInit("/home/robbe/calib.mp4", 60);
}

void mousePressed() {
  dragStart = new PVector(mouseX, mouseY);
  for (Node node : n) {
    if (node.p.dist(mouse) < node.d) {
      dragged = node;
      return;
    }
  }
}

void mouseDragged() {
  mouseDragged = true;
  if (dragged != null) {
    dragged.p.set(mouse);
  }
}

void mouseReleased() {
  mouseDragged = false;
  dragged = null;
}

void updateRough() {
  // set weights to actual distances
  for (Edge edge : e) {
    edge.w = edge.a.p.dist(edge.b.p);
  }
}

void updateFine(float dt) {
  // reset s in all nodes
  for (Node node : n) {
    node.f.set(0, 0);
  }

  // calc forces across all edges
  for (Edge edge : e) {
    PVector f = edge.force();
    edge.a.f.add(f);
    edge.b.f.sub(f); // actio == reactio!
  }

  // apply forces to all nodes
  for (Node node : n) {
    PVector dp = node.f.copy().mult(dt);
    if (node != dragged) { 
      node.p.add(dp);
    }
  }
}

void cycleMode() {
  if (mode == Mode.DONE) mode = Mode.ROUGH; 
  else if (mode == Mode.ROUGH) mode = Mode.FINE;
  else if (mode == Mode.FINE) mode = Mode.DONE;
  log("Switched to mode " + mode);
}

void keyPressed() {  
  switch (key) {  
  case 's':
    store("target.csv");    
    break;

  case 'l':
    load("target.csv");    
    break;

  case 'm':
    cycleMode();
    break;

  case 'n':
    currentEdge = (currentEdge + 2) % (e.length + 1) - 1;
    log("Selected edge " + currentEdge);
    break;

  case '0':
  case '1':
  case '2':
  case '3':
  case '4':
  case '5':
  case '6':
  case '7':
  case '8':
  case '9':    
    if (currentEdge >= 0) {
      currentLength += key;
    }
    break;

  case '+':
    if (currentEdge >= 0) {
      e[currentEdge].w *= 1.1;
    }
    break;

  case '-':
    if (currentEdge >= 0) {
      e[currentEdge].w /= 1.1;
    }
    break;

  case ENTER:
  case RETURN:    
    if (currentEdge >= 0) {
      float input = float(currentLength);
      e[currentEdge].groundTruth(input);      
      currentLength = "";
      log("Set edge " + currentEdge + " to " + currentLength + "mm");
    }
    break;
  }
}

void store(String fname) {
  Table table = new Table();

  table.addColumn("x");
  table.addColumn("y");

  for (Node node : n) {
    TableRow row = table.addRow();
    row.setFloat("x", node.p.x);
    row.setFloat("y", node.p.y);
  }

  saveTable(table, fname);
  log("stored calibration to " + fname);
}

void load(String fname) {
  Table table = loadTable(fname, "header");
  int i=0;

  for (TableRow row : table.rows()) {
    n[i].p.x = row.getFloat("x");
    n[i].p.y = row.getFloat("y");
    i++;
  }

  log("stored calibration from " + fname);
}

void log(String msg) {  
  logs.add(msg);
  logMillis.add(millis());
  println(msg);
}

// extracts screen space coords of nodes to 2D array
double[][] getCoords(Node[] nodes, boolean screenSpace) {
  int n = nodes.length;
  double[][] x = new double[n][];

  for (int i=0; i<n; i++) {
    PVector v = screenSpace ? nodes[i].p : nodes[i].q;
    x[i] = new double[]{v.x, v.y};
  }

  return x;
}

double[][] findHomography(double[][] x, double[][] X) {
  // equation system to solve for h
  double[][] A = new double[8][];
  double[][] b = new double[8][];
  for (int i=0; i<4; i++) {
    // coefficients
    A[2*i+0] = new double[] { x[i][0], x[i][1], 1, 0, 0, 0, -X[i][0]*x[i][0], -X[i][0]*x[i][1] }; 
    A[2*i+1] = new double[] { 0, 0, 0, x[i][0], x[i][1], 1, -X[i][1]*x[i][0], -X[i][1]*x[i][1] };
    // constant terms
    b[2*i+0] = new double[] { X[i][0] };
    b[2*i+1] = new double[] { X[i][1] };
  };    

  // solve Ah = b
  double[][] h = new LUD(A.clone()).solve(b);

  // reshape
  double[][] H = {
    { h[0][0], h[1][0], h[2][0] }, 
    { h[3][0], h[4][0], h[5][0] }, 
    { h[6][0], h[7][0], 1 }
  };

  return H;
}

public static String str(double[][] M) {
  StringBuilder sb = new StringBuilder();
  for (int i = 0; i < M.length; i++) {
    for (int j = 0; j < M[i].length; j++) {
      sb.append('\t').append(M[i][j]);
    }
    sb.append('\n');
  }
  return sb.toString();
}

public static double[][] multiply(double[][] A, double[][] B) {
  int Am = A.length, An = A[0].length;
  int Bm = B.length, Bn = B[0].length;

  if (Bm != An) {
    throw new IllegalArgumentException("Matrix inner dimensions must agree.");
  }
  double[][] C = new double[Am][Bn];
  double[] Bcolj = new double[An];
  for (int j = 0; j < Bn; j++) {
    for (int k = 0; k < An; k++) {
      Bcolj[k] = B[k][j];
    }
    for (int i = 0; i < Am; i++) {
      double[] Arowi = A[i];
      double s = 0;
      for (int k = 0; k < An; k++) {
        s += Arowi[k] * Bcolj[k];
      }
      C[i][j] = s;
    }
  }
  return C;
}

void drawHandles() {
  // draw edges
  switch (mode) {

  case ROUGH:
    for (Edge edge : e) {
      edge.draw(false, true);
    }
    break;

  case FINE: 
    if (currentEdge >= 0) {
      e[currentEdge].draw(true);
    }
    break;

  default:
    break;
  }

  // draw nodes
  for (Node node : n) {    
    node.draw(dragged == node);
  }
}

void renderGrid(float len, int n) {
  float step = len / n;
  for (int i=0; i<=n; i++) {
    line(-len/2, (i-n/2)*step, len/2, (i-n/2)*step);
    line((i-n/2)*step, -len/2, (i-n/2)*step, len/2);
    //line(i*step, -len/2, i*step, len/2);
  }
}


PVector transform(PVector v, double[][] H) {
  double[][] x = {{v.x}, {v.y}, {1}};
  double[][] X = multiply(H, x);
  return new PVector((float)(X[0][0]/X[2][0]), (float)(X[1][0]/X[2][0]));
}

// physical coordinates here!
void render() {  
  int alpha = (mode == Mode.DONE) ? 128 : 32;

  stroke(92, alpha);
  strokeWeight(1);
  renderGrid(10000, 1000);

  stroke(128, alpha);
  strokeWeight(2);
  renderGrid(10000, 100);

  stroke(255, alpha);
  strokeWeight(3);
  renderGrid(3000, 3);

  fill(255);

  if (mousePressed && mode == Mode.DONE) {
    pushMatrix();
    PVector pointer = transform(mouse, Hinv);
    textSize(50);
    textAlign(CENTER, TOP);
    if (mouseDragged) {
      PVector start = transform(dragStart, Hinv);
      PVector v = pointer.copy().sub(start);


      stroke(255);
      line(start.x, start.y, pointer.x, pointer.y);

      float angle = v.heading();

      translate(start.x, start.y);

      rotate(angle);
      translate(v.mag()/2, 0);
      text(round(v.mag())+"", 0, 0);
    } else {
      text("("+round(pointer.x)+","+round(pointer.y)+")", pointer.x, pointer.y);
    }
    popMatrix();
  }

  //scale(18.489 * 25.4 / 1926);
  //image(img,0,0);
}

void drawGrid(float len, int n, double[][] H) {
  float step = len / n;
  for (int i=0; i<=n; i++) {
    PVector a = transform(new PVector(-len/2, (i-n/2)*step), H);
    PVector b = transform(new PVector(len/2, (i-n/2)*step), H);

    // uncondition
    a.x += center[0]; 
    a.y += center[1];
    b.x += center[0]; 
    b.y += center[1];

    line(a.x, a.y, b.x, b.y);

    if (i == 0)
      println("("+a.x+","+a.y+")("+b.x+","+b.y+")");

    ellipse(a.x, a.y, 10, 10);

    //line(i*step, -len/2, i*step, len/2);
  }
}

double[][] add(double[][] points, double[] v) {
  int n = points.length;
  double[][] dst = points.clone();
  for (int i=0; i<n; i++) {
    dst[i][0] += v[0];
    dst[i][1] += v[1];
  }
  return dst;
}

double[][] sub(double[][] points, double[] v) {
  int n = points.length;
  double[][] dst = points.clone();
  for (int i=0; i<n; i++) {
    dst[i][0] -= v[0];
    dst[i][1] -= v[1];
  }
  return dst;
}

double[][] mult(double[][] points, double v) {
  int n=points.length;
  double[][] dst = points.clone();
  for (int i=0; i<n; i++) {
    points[i][0] *= v;
    points[i][1] *= v;
  }
  return dst;
}

double[] mean(double[][] points) {
  int n=points.length;
  double[] c = new double[2];

  for (int i=0; i<n; i++) {
    c[0] += points[i][0];
    c[1] += points[i][1];
  }

  return c;
}

void draw() {  
  float millis = millis();
  float dt = millis - pMillis; 

  mouse.set(mouseX, mouseY);    

  switch (mode) {
  case ROUGH:
    updateRough();
    break;
  case FINE:
    updateFine(dt);
    break;
  default:
    break;
  }

  // drop old console messages
  Integer t = null;
  while ((t = logMillis.peek()) != null) {
    if (millis() - t > 5000) {
      logMillis.poll();
      logs.poll();
    } else {
      break;
    }
  }

  // calc FPS
  fps =  .9 * fps + .1 * round(1000./dt);

  // render
  background(0);
  strokeWeight(1);
  fill(255);

  // draw HUD layer   
  textAlign(RIGHT, TOP);
  text("fps:"+round(fps)+ " ", width, 0);

  // draw console
  fill(255);
  textAlign(LEFT, TOP);
  int i=0;
  for (String msg : logs) {
    text(msg, 0, 10*i++);
  }

  // draw handles
  if (mode != Mode.DONE) {
    drawHandles();
  }

  // draw cursor
  fill(255, 64);
  ellipse(mouseX, mouseY, 1, 1);

  // find homography (and inverse)
  double[][] screenCoords = getCoords(n, true);
  double[][] spaceCoords = getCoords(n, false);

  // conditioning
  //center = mean(screenCoords);
  //screenCoords = sub(screenCoords, center);  

  H = findHomography(spaceCoords, screenCoords);
  Hinv = findHomography(screenCoords, spaceCoords);

  //drawGrid(3000, 3, H);

  // warp to physical space
  applyMatrix(  (float)H[0][0], (float)H[0][1], 0, (float)H[0][2], 
    (float)H[1][0], (float)H[1][1], 0, (float)H[1][2], 
    0, 0.0, 1, 0.0, 
    (float)H[2][0], (float)H[2][1], 0, (float)H[2][2]);


  // render in physical coordiante system
  pushStyle();
  render();  
  popStyle();

  //ffmpegCapture(); // screen cap for video

  pMillis = millis;
}
