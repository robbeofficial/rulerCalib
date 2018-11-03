class Vector {
    constructor(x = 0, y = 0, z = 0) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    get length() { return Math.sqrt(this.x ** 2 + this.y ** 2 + this.z ** 2); }

    add(v) { return new Vector(this.x + v.x, this.y + v.y, this.z + v.z); }
    mult(s) { return new Vector(this.x * s, this.y * s, this.z * s); }

    sub(v) { return this.add(v.mult(-1)); }
    div(s) { return this.mult(1/s); }

    dist(v) { return(this.sub(v).length); }
}

class Node {
    constructor(u, v, x, y, callback=function(){}) {
        this.p = new Vector(u, v);  // position in screen space
        this.q = new Vector(x, y); // position in physical space
        this.f = new Vector(); // current force on this node
        this.d = 8; // diameter
        this.callback = callback;
    }

    set screenPos(v) {
        this.p = v;
        this.callback(this);
    }
}

class Edge {
    constructor(a, b, w) {
        this.a = a;
        this.b = b;
        this.w = w; // weight = desired distance in screen space
    }      

    force() {        
        const mu = .005; // friction
        let v = this.b.p.sub(this.a.p); // v = b - a 
        let d = v.length; // d = |v|  
        let f = (d - this.w) * mu; // f = (d - w) * µ 
        return v.div(d).mult(f);
    }

    get length() {
        return this.a.q.dist(this.b.q);
    }

    set length(measurement) {        
        this.w *= this.length / measurement;
    }
}

class Graph {
    constructor() {
        this.nodes = [];
        this.edges = [];
    }

    updateWeights() {
        // set weights to actual distances
        this.edges.forEach(function (edge) {
          edge.w = edge.a.p.dist(edge.b.p);
        });
    }

    updateForces(dt) {
        // reset s in all nodes
        this.nodes.forEach(function (node) {
            node.f = new Vector();
        });

        // calc forces across all edges
        this.edges.forEach(function (edge) {
            let f = edge.force();
            edge.a.f = edge.a.f.add(f);
            edge.b.f = edge.b.f.sub(f); // actio == reactio!
        });

        // apply forces to all nodes        
        this.nodes.forEach(function (node) {
            let dp = node.f.mult(dt);
            node.screenPos = node.p.add(dp);
        });
    }
}