class Node {
  PVector p; // position in screen space
  PVector q; // position in physical space
  PVector f; // current force on this node
  float d = 8; // diameter

  public Node(float u, float v, float x, float y) {
    p = new PVector(u, v);
    q = new PVector(x, y);
    f = new PVector();
  }

  void draw(boolean highlighted) {
    ellipseMode(CENTER);
    stroke(255);
    if (highlighted) {
      fill(255);      
    } else {
      noFill();
    }
    ellipse(p.x, p.y, d, d);
  }
  
  void draw() {
    draw(false);  
  }
}

class Edge {
  Node a, b;
  float w; // weight = desired distance in screen space

  public Edge(Node a, Node b, float w) {
    this.a = a;
    this.b = b;
    this.w = w;
  }
  
  // provides ground truth input on this edge (as measured in physical space)
  public void groundTruth(float measurement) {
    float expected = a.q.dist(b.q);
    w *= expected / measurement;
  }

  public void draw(boolean highlighted, boolean weight) {
    // high shade for stressed edges
    float d = a.p.dist(b.p);
    float blink = (millis() / 50. % 5) / 5; 
    float shade = abs(d - w) / height * 255. * 10;
     
    if (highlighted) {
      stroke(255 * blink);  
    } else {
      stroke(shade, 255-shade, 0);
    }
    
    // draw edge
    line(a.p.x, a.p.y, b.p.x, b.p.y);
    
    // draw edge weight and physical distance on edge center
    if (weight) {
      float dq = a.q.dist(b.q);
      PVector p = a.p.copy().add(b.p).div(2);
      textAlign(CENTER);
      text(round(d)+"px\n"+round(dq*10)/10., p.x, p.y);
    }
  }
  
  PVector force() {
    final float mu = .005; // friction
    PVector v = b.p.copy().sub(a.p); // v = b - a 
    float d = v.mag(); // d = |v|  
    float f = (d - w) * mu; // f = (d - w) * µ 
    return v.div(d).mult(f);
  }
  
  public void draw() {
    draw(false, false);  
  }
  
  public void draw(boolean highlighted) {
    draw(highlighted, false);
  }
}
