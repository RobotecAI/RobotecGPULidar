## Usage

RGL was designed to be easy to use from any programming language, therefore it exposes a low-level C API.

The C API is built around several concepts that are introduced in the following chapter.

## API concepts

### Mesh

Mesh is a handle to the on-GPU data of a 3D model provided by user.

### Entity

Entity represents a 3D object on the scene along with its pose (position and rotation).
Entity is a lightweight object - it contains a reference to a heavier Mesh object and a transformation matrix.
Entity's transform is a 3D affine matrix (therefore 3x4) describing entity's local to scene coordinate frame transform.
When created, entity is bound to a scene. Entity cannot be unbound from a scene or bound to multiple scenes.
To create an Entity it is required to provide a Mesh, which must be created first.
Entities can share the same mesh.

### Scene

Scene represents 'a place' where the raytracing occurs.
User is expected to fill a scene with entities before performing ray-tracing.
Internally it is used to build hardware raytracing acceleration structures.
Most of the time, a single scene will be sufficient, therefore RGL automatically instantiates the default scene.
The default scene can be referenced by passing null pointer where scene handle is expected.

Note: using multiple scenes is not yet implemented.

### Node

Performs a single operation, for example:
 - set rays for raytracing from transforms 
 - transform rays
 - set the desired output format 
 - compact result (remove non-hits)
 - downsample result (merge points that are very close to each other)
 - perform raytracing

It has to be connected to other Nodes in order to function properly.
If a node is active, it is executed while running the Graph.
Nodes are active by default, but can be deactivated.
Children of deactivated Nodes are also not executed while running the Graph.

### Graph

Connected Nodes create a Graph. The Graph can be run to calculate the result for each node.
Using the graph concept the end user can easily tailor the functionality and output format to their needs by adding / removing certain Nodes.
The typical use-case is simulating a Lidar.

## General approach

Usually, using the library will consist of the following steps:

1. Create Meshes (e.g. use an external tool to read them from a file)
2. Create Entities on the scene
3. Create Nodes
4. Connect Nodes into Graph(s)
5. Set Entities' poses
6. Run Graph(s)

RGL is optimized to be used in ever-changing scenes, therefore it is possible to repeat steps 4-6 dozens of times per second (may vary depending on the number of entites and total number of rays).

## Minimal example

The minimal example below demonstrates ray-tracing of a single ray on a scene consisting of a single cube.
Full source code can be found [here](../test/src/apiReadmeExample.cpp)

![Diagram of the example scene](image/readme-example-scene.svg)

```c
// Create a mesh
rgl_mesh_t cube_mesh = makeCubeMesh();

// Put an entity on the default scene
rgl_entity_t cube_entity = 0;
EXPECT_RGL_SUCCESS(rgl_entity_create(&cube_entity, NULL, cube_mesh));

// Set position of the cube entity to (0, 0, 5)
rgl_mat3x4f entity_tf = {
.value = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 5 } }
};
EXPECT_RGL_SUCCESS(rgl_entity_set_pose(cube_entity, &entity_tf));

// Create a graph representation of a lidar that sends 1 ray and is situated at (x,y,z) = (0, 0, 0), facing positive Z
rgl_mat3x4f ray_tf = {
.value = {
    { 1, 0, 0, 0 },
    { 0, 1, 0, 0 },
    { 0, 0, 1, 0 },}
};

rgl_node_t useRays = nullptr, raytrace = nullptr;

EXPECT_RGL_SUCCESS(rgl_node_rays_from_mat3x4f(&useRays, &ray_tf, 1));
EXPECT_RGL_SUCCESS(rgl_node_raytrace(&raytrace, nullptr, 1000));
EXPECT_RGL_SUCCESS(rgl_graph_node_add_child(useRays, raytrace));

// you can run the graph using any one of its nodes
EXPECT_RGL_SUCCESS(rgl_graph_run(raytrace));

// Wait for the Graph to run (if needed) and collect results
size_t hitpoint_count = 0;
size_t size;
rgl_vec3f results[1] = { 0 };
EXPECT_RGL_SUCCESS(rgl_graph_get_result_size(raytrace, RGL_FIELD_XYZ_F32, &hitpoint_count, &size));
EXPECT_RGL_SUCCESS(rgl_graph_get_result_data(raytrace, RGL_FIELD_XYZ_F32, &results));

printf("Got %ld hitpoint(s)\n", hitpoint_count);
for (int i = 0; i < hitpoint_count; ++i) {
printf("- (%.2f, %.2f, %.2f)\n", results[i].value[0], results[i].value[1], results[i].value[2]);
}
```

### API documentation

More details can be found [here](../include/rgl/api/experimental.h).
