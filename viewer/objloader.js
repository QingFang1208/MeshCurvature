function ObjLoader(text) {
    const vertices = [];
    const vertexColors = [];
    const indices = []; // array of face indices (triangles)
    const vertexNormals = [];
  
    // Store vertices and optional colors
    const rawVerts = [];
    const vertColors = [];
    vertNormals = [];
  
    const lines = text.split('\n');
  
    for (const line of lines) {
      const parts = line.trim().split(/\s+/);
      if (parts[0] === 'v') {
        const x = parseFloat(parts[1]);
        const y = parseFloat(parts[2]);
        const z = parseFloat(parts[3]);
        rawVerts.push([x, y, z]);
  
        if (parts.length >= 7) {
          const r = parseFloat(parts[4]);
          const g = parseFloat(parts[5]);
          const b = parseFloat(parts[6]);
          vertColors.push([r, g, b]);
        } else {
          vertColors.push([0.11, 0.39, 0.89]); // default blue
        }
      }
  
      if (parts[0] === 'f') {
        // Parse triangle face, e.g. f 1 2 3
        const faces = parts.slice(1).map(p => parseInt(p.split('/')[0], 10) - 1);
        if (faces.length === 3) {
          indices.push(faces);
        } else if (faces.length === 4) {
          // convert quad to two triangles
          indices.push([faces[0], faces[1], faces[2]]);
          indices.push([faces[0], faces[2], faces[3]]);
        }
      }

      if (parts[0] === 'vn'){
        const vx = parseFloat(parts[1]);
        const vy = parseFloat(parts[2]);
        const vz = parseFloat(parts[3]);
        vertNormals.push([vx, vy, vz]);
      }
    }
  
    // Initialize empty normals per vertex
    if (vertNormals.length !== rawVerts.length){
        vertNormals = Array(rawVerts.length).fill().map(() => [0, 0, 0]);
        // Compute face normals and accumulate for each vertex
        for (const [i0, i1, i2] of indices) {
        const v0 = rawVerts[i0];
        const v1 = rawVerts[i1];
        const v2 = rawVerts[i2];
    
        const edge1 = subtract(v1, v0);
        const edge2 = subtract(v2, v0);
        const faceNormal = normalize(cross(edge1, edge2));
    
        addTo(vertNormals[i0], faceNormal);
        addTo(vertNormals[i1], faceNormal);
        addTo(vertNormals[i2], faceNormal);
        }
    }
    
    // Normalize the accumulated normals
    const finalNormals = vertNormals.map(n => normalize(n));
  
    // Flatten everything for rendering
    const outvertices = [];
    const outvertexColors = [];
    const outvertexNormals = [];
    const outIndices = [];
  
    for (let i = 0; i < rawVerts.length; ++i) {
      outvertices.push(...rawVerts[i]);
      outvertexColors.push(...vertColors[i]);
      outvertexNormals.push(...finalNormals[i]);
    }
  
    for (const tri of indices) {
      outIndices.push(...tri);
    }
  
    return {
      vertices: outvertices,
      vertexColors: outvertexColors,
      vertexNormals: outvertexNormals,
      indices: outIndices
    };
  }

  function subtract(a, b) {
    return [a[0]-b[0], a[1]-b[1], a[2]-b[2]];
  }
  
  function cross(a, b) {
    return [
      a[1]*b[2] - a[2]*b[1],
      a[2]*b[0] - a[0]*b[2],
      a[0]*b[1] - a[1]*b[0]
    ];
  }
  
  function normalize(v) {
    const len = Math.sqrt(v[0]**2 + v[1]**2 + v[2]**2);
    return len > 1e-5 ? [v[0]/len, v[1]/len, v[2]/len] : [0, 0, 0];
  }
  
  function addTo(a, b) {
    a[0] += b[0]; a[1] += b[1]; a[2] += b[2];
  }
  