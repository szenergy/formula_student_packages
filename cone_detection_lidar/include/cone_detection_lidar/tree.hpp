#ifndef TREE_HPP_
#define TREE_HPP_

#include <cmath>
#include <vector>
#include <algorithm>
#include <functional>
#include <iostream>
#include <stack>
#include "pointcloud_to_grid_core.hpp"

class Node
{
public:
    PointXY pos;
    float prob = 0.5;
    std::vector<Node *> children;
    Node(PointXY pos) : pos(pos) {}
};

class Tree
{

public:
    // create a root node and reserve memory for it
    Node *root = new Node(PointXY(0.0, 0.0));

    Node *getRoot()
    {
        return root;
    }

    Node *modifyRoot(PointXY pos)
    {
        root->pos = pos;
        return root;
    }

    Node *modifyNode(Node *node, PointXY pos)
    {
        node->pos = pos;
        return node;
    }

    Node *addNode(Node *parent, PointXY pos)
    {
        Node *newNode = new Node(pos);
        parent->children.push_back(newNode);
        return newNode;
    }
    // recursive function to visit all nodes in the tree
    void printTree(Node *node, std::string indent = "")
    {
        if (node != nullptr)
        {
            if (root == node)
            {
                // skip printing root node
            }
            else
            {
                std::cout << indent << "(" << node->pos.x << ", " << node->pos.y << ")" << std::endl;
            }
            for (Node *child : node->children)
            {
                Node *father = node;
                std::cout << "[" << father->pos.x << ", " << father->pos.y << "] ";
                printTree(child, indent + "->");
            }
        }
    }
    // alternative way to print (and walk thru) the whole tree (non-recursive)
    void printTreeAlt(Node *root)
    {
        if (root == nullptr)
        {
            return;
        }

        std::stack<Node *> stack;
        stack.push(root);

        while (!stack.empty())
        {
            Node *node = stack.top();
            stack.pop();

            if (node != root)
            {
                std::cout << "(" << node->pos.x << ", " << node->pos.y << ")" << std::endl;
            }

            for (Node *child : node->children)
            {
                std::cout << "[" << node->pos.x << ", " << node->pos.y << "] ";
                stack.push(child);
            }
        }
    }

    // recursive function to visualize the tree in rviz
    void markerTree(Node *node, visualization_msgs::msg::Marker &line1_marker, std::string indent = "")
    {
        if (node != nullptr)
        {
            if (root == node)
            {
                // skip root node
            }
            else
            {
                geometry_msgs::msg::Point p_node;
                p_node.x = node->pos.x;
                p_node.y = node->pos.y;
                p_node.z = 0.0;
                line1_marker.points.push_back(p_node);
            }
            for (Node *child : node->children)
            {
                Node *father = node;
                geometry_msgs::msg::Point p_father;
                p_father.x = father->pos.x;
                p_father.y = father->pos.y;
                p_father.z = 0.0;
                line1_marker.points.push_back(p_father);
                markerTree(child, line1_marker, indent + "->");
            }
        }
    }
};

#endif // TREE_HPP_
