clear; clc;
addpath('Core');

node = Node(true, "test");
pnode = PhysicsNode();
pnodechild = PhysicsNode();
pnodechild.recordableVariables = ["mass", "inertiaTensor"];
pnodechild.name = "test";

node.addChild(pnode);
pnode.addChild(pnodechild);