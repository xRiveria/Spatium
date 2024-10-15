#include "BVH/BVH.hpp"

#include <GLM/gtc/matrix_transform.hpp>
#include <iostream>

 struct Object;

 using BVHObject = Spatium::BVH<Object*>;
 using BVHNode = BVHObject::BVHNode;

 // Scene objects. The BVH contains leaf nodes storing these.
 struct Object
 {
     Object(uint32_t objectID, const Spatium::AABB& objectAABB, int meshIndex, const glm::mat4& worldMatrix) : m_ID(objectID), m_AABB(objectAABB), m_MeshIndex(meshIndex), m_WorldMatrix(worldMatrix) { }

     uint32_t m_ID;         // Object identification UUID.
     Spatium::AABB m_AABB; // Bounding volume of the object
     int m_MeshIndex;  // Index to a list of primitives.
     glm::mat4 m_WorldMatrix;        // As loaded from the scene file.

     // BVH information
     struct
     {
         Object* m_Next = nullptr; // Next object in the BVH node.
         Object* m_Previous = nullptr; // Previous object in the BVH node.
         BVHNode* m_Node = nullptr; // The node it belongs to.
     } m_BVHInfo;
 };

 void GenerateDummyObjects(std::vector<std::shared_ptr<Object>>& sceneObjects);


int main()
{
    std::vector<std::shared_ptr<Object>> sceneObjects;
    GenerateDummyObjects(sceneObjects);

    std::vector<Object*> objectPtrs;
    for (const auto& sceneObject : sceneObjects)
    {
        objectPtrs.push_back(sceneObject.get());
    }

    Spatium::BVHBuildConfiguration buildConfiguration;
    buildConfiguration.m_MinimumObjects = 5;
    buildConfiguration.m_MinimumVolume = 10;
  
    BVHObject objectBVH;

    objectBVH.Clear();
    objectBVH.BuildTopDown(objectPtrs.begin(), objectPtrs.end(), buildConfiguration);
    objectBVH.Clear();
    objectBVH.BuildBottomUp(objectPtrs.begin(), objectPtrs.end(), buildConfiguration);
    objectBVH.Clear();
    objectBVH.Insert(objectPtrs.begin(), objectPtrs.end(), buildConfiguration);

    std::cout << "Hello World!\n";
}

void GenerateDummyObjects(std::vector<std::shared_ptr<Object>>& sceneObjects)
{
    sceneObjects.emplace_back(std::make_shared<Object>(101, Spatium::AABB(glm::vec3(0.0f, 0.0f, 0.0f), glm::vec3(1.0f, 1.0f, 1.0f)), 5, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(102, Spatium::AABB(glm::vec3(-2.0f, -2.0f, -2.0f), glm::vec3(0.0f, 0.0f, 0.0f)), 7, glm::mat4(0.5f)));
    sceneObjects.emplace_back(std::make_shared<Object>(103, Spatium::AABB(glm::vec3(1.0f, 1.0f, 1.0f), glm::vec3(2.0f, 2.0f, 2.0f)), 2, glm::translate(glm::mat4(1.0f), glm::vec3(2.0f, 0.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(104, Spatium::AABB(glm::vec3(-3.0f, -3.0f, 0.0f), glm::vec3(1.0f, 1.0f, 4.0f)), 10, glm::scale(glm::mat4(1.0f), glm::vec3(1.5f, 1.5f, 1.5f))));
    sceneObjects.emplace_back(std::make_shared<Object>(105, Spatium::AABB(glm::vec3(-1.0f, -1.0f, -1.0f), glm::vec3(1.0f, 2.0f, 3.0f)), 12, glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(106, Spatium::AABB(glm::vec3(10.0f, 5.0f, 3.0f), glm::vec3(12.0f, 7.0f, 5.0f)), 1, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(107, Spatium::AABB(glm::vec3(-10.0f, -5.0f, -3.0f), glm::vec3(-9.0f, -4.0f, -2.0f)), 3, glm::mat4(0.8f)));
    sceneObjects.emplace_back(std::make_shared<Object>(108, Spatium::AABB(glm::vec3(30.0f, 20.0f, 15.0f), glm::vec3(32.0f, 22.0f, 17.0f)), 6, glm::translate(glm::mat4(1.0f), glm::vec3(3.0f, 2.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(109, Spatium::AABB(glm::vec3(50.0f, 0.0f, -50.0f), glm::vec3(53.0f, 3.0f, -47.0f)), 9, glm::scale(glm::mat4(1.0f), glm::vec3(1.2f, 1.2f, 1.2f))));
    sceneObjects.emplace_back(std::make_shared<Object>(110, Spatium::AABB(glm::vec3(-20.0f, -15.0f, -10.0f), glm::vec3(-18.0f, -13.0f, -8.0f)), 11, glm::rotate(glm::mat4(1.0f), glm::radians(30.0f), glm::vec3(0.0f, 0.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(111, Spatium::AABB(glm::vec3(100.0f, 50.0f, 75.0f), glm::vec3(105.0f, 55.0f, 80.0f)), 14, glm::mat4(0.7f)));
    sceneObjects.emplace_back(std::make_shared<Object>(112, Spatium::AABB(glm::vec3(-100.0f, -50.0f, -75.0f), glm::vec3(-98.0f, -48.0f, -73.0f)), 16, glm::scale(glm::mat4(1.0f), glm::vec3(0.9f, 0.9f, 0.9f))));
    sceneObjects.emplace_back(std::make_shared<Object>(113, Spatium::AABB(glm::vec3(200.0f, 150.0f, 120.0f), glm::vec3(203.0f, 153.0f, 123.0f)), 18, glm::rotate(glm::mat4(1.0f), glm::radians(90.0f), glm::vec3(1.0f, 0.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(114, Spatium::AABB(glm::vec3(300.0f, 200.0f, 150.0f), glm::vec3(305.0f, 205.0f, 155.0f)), 21, glm::translate(glm::mat4(1.0f), glm::vec3(5.0f, 5.0f, 5.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(115, Spatium::AABB(glm::vec3(-150.0f, -120.0f, -100.0f), glm::vec3(-148.0f, -118.0f, -98.0f)), 24, glm::rotate(glm::mat4(1.0f), glm::radians(60.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(116, Spatium::AABB(glm::vec3(0.0f, 0.0f, 300.0f), glm::vec3(5.0f, 5.0f, 305.0f)), 26, glm::scale(glm::mat4(1.0f), glm::vec3(1.1f, 1.1f, 1.1f))));
    sceneObjects.emplace_back(std::make_shared<Object>(117, Spatium::AABB(glm::vec3(50.0f, -50.0f, 50.0f), glm::vec3(55.0f, -45.0f, 55.0f)), 28, glm::rotate(glm::mat4(1.0f), glm::radians(120.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(118, Spatium::AABB(glm::vec3(-50.0f, -50.0f, -50.0f), glm::vec3(-47.0f, -47.0f, -47.0f)), 30, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(119, Spatium::AABB(glm::vec3(500.0f, 400.0f, 300.0f), glm::vec3(505.0f, 405.0f, 305.0f)), 32, glm::mat4(0.6f)));
    sceneObjects.emplace_back(std::make_shared<Object>(120, Spatium::AABB(glm::vec3(-200.0f, -300.0f, -400.0f), glm::vec3(-198.0f, -298.0f, -398.0f)), 35, glm::translate(glm::mat4(1.0f), glm::vec3(10.0f, 10.0f, 10.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(121, Spatium::AABB(glm::vec3(1000.0f, 1000.0f, 1000.0f), glm::vec3(1005.0f, 1005.0f, 1005.0f)), 38, glm::scale(glm::mat4(1.0f), glm::vec3(2.0f, 2.0f, 2.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(122, Spatium::AABB(glm::vec3(-1000.0f, -1000.0f, -1000.0f), glm::vec3(-995.0f, -995.0f, -995.0f)), 41, glm::rotate(glm::mat4(1.0f), glm::radians(150.0f), glm::vec3(1.0f, 0.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(123, Spatium::AABB(glm::vec3(150.0f, 75.0f, 60.0f), glm::vec3(155.0f, 80.0f, 65.0f)), 43, glm::translate(glm::mat4(1.0f), glm::vec3(7.0f, 3.0f, 2.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(124, Spatium::AABB(glm::vec3(20.0f, -30.0f, 10.0f), glm::vec3(25.0f, -25.0f, 15.0f)), 46, glm::rotate(glm::mat4(1.0f), glm::radians(180.0f), glm::vec3(0.0f, 1.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(125, Spatium::AABB(glm::vec3(-600.0f, -500.0f, -400.0f), glm::vec3(-595.0f, -495.0f, -395.0f)), 48, glm::scale(glm::mat4(1.0f), glm::vec3(0.8f, 0.8f, 0.8f))));
    sceneObjects.emplace_back(std::make_shared<Object>(126, Spatium::AABB(glm::vec3(350.0f, 250.0f, 150.0f), glm::vec3(355.0f, 255.0f, 155.0f)), 51, glm::rotate(glm::mat4(1.0f), glm::radians(45.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(127, Spatium::AABB(glm::vec3(0.0f, 400.0f, -300.0f), glm::vec3(5.0f, 405.0f, -295.0f)), 53, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(128, Spatium::AABB(glm::vec3(-250.0f, -250.0f, -250.0f), glm::vec3(-245.0f, -245.0f, -245.0f)), 56, glm::translate(glm::mat4(1.0f), glm::vec3(15.0f, 0.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(129, Spatium::AABB(glm::vec3(450.0f, 350.0f, 250.0f), glm::vec3(455.0f, 355.0f, 255.0f)), 58, glm::rotate(glm::mat4(1.0f), glm::radians(15.0f), glm::vec3(0.0f, 0.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(130, Spatium::AABB(glm::vec3(300.0f, 100.0f, 400.0f), glm::vec3(305.0f, 105.0f, 405.0f)), 61, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(131, Spatium::AABB(glm::vec3(-800.0f, -900.0f, -1000.0f), glm::vec3(-795.0f, -895.0f, -995.0f)), 63, glm::scale(glm::mat4(1.0f), glm::vec3(1.5f, 1.5f, 1.5f))));
    sceneObjects.emplace_back(std::make_shared<Object>(132, Spatium::AABB(glm::vec3(700.0f, 600.0f, 500.0f), glm::vec3(705.0f, 605.0f, 505.0f)), 65, glm::rotate(glm::mat4(1.0f), glm::radians(135.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(133, Spatium::AABB(glm::vec3(-600.0f, 100.0f, 200.0f), glm::vec3(-595.0f, 105.0f, 205.0f)), 68, glm::translate(glm::mat4(1.0f), glm::vec3(20.0f, 10.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(134, Spatium::AABB(glm::vec3(900.0f, 800.0f, 700.0f), glm::vec3(905.0f, 805.0f, 705.0f)), 70, glm::rotate(glm::mat4(1.0f), glm::radians(75.0f), glm::vec3(1.0f, 0.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(135, Spatium::AABB(glm::vec3(350.0f, 250.0f, 50.0f), glm::vec3(355.0f, 255.0f, 55.0f)), 73, glm::scale(glm::mat4(1.0f), glm::vec3(0.5f, 0.5f, 0.5f))));
    sceneObjects.emplace_back(std::make_shared<Object>(136, Spatium::AABB(glm::vec3(100.0f, -100.0f, 0.0f), glm::vec3(105.0f, -95.0f, 5.0f)), 75, glm::translate(glm::mat4(1.0f), glm::vec3(25.0f, 25.0f, 25.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(137, Spatium::AABB(glm::vec3(500.0f, 600.0f, 700.0f), glm::vec3(505.0f, 605.0f, 705.0f)), 78, glm::rotate(glm::mat4(1.0f), glm::radians(105.0f), glm::vec3(0.0f, 0.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(138, Spatium::AABB(glm::vec3(-450.0f, -350.0f, -250.0f), glm::vec3(-445.0f, -345.0f, -245.0f)), 80, glm::scale(glm::mat4(1.0f), glm::vec3(0.3f, 0.3f, 0.3f))));
    sceneObjects.emplace_back(std::make_shared<Object>(139, Spatium::AABB(glm::vec3(200.0f, 100.0f, 50.0f), glm::vec3(205.0f, 105.0f, 55.0f)), 83, glm::rotate(glm::mat4(1.0f), glm::radians(165.0f), glm::vec3(0.0f, 0.0f, 1.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(140, Spatium::AABB(glm::vec3(600.0f, 700.0f, 800.0f), glm::vec3(605.0f, 705.0f, 805.0f)), 85, glm::mat4(1.0f)));
    sceneObjects.emplace_back(std::make_shared<Object>(141, Spatium::AABB(glm::vec3(300.0f, 200.0f, 400.0f), glm::vec3(305.0f, 205.0f, 405.0f)), 88, glm::rotate(glm::mat4(1.0f), glm::radians(195.0f), glm::vec3(1.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(142, Spatium::AABB(glm::vec3(900.0f, 1000.0f, 1100.0f), glm::vec3(905.0f, 1005.0f, 1105.0f)), 90, glm::translate(glm::mat4(1.0f), glm::vec3(30.0f, 20.0f, 10.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(143, Spatium::AABB(glm::vec3(100.0f, -200.0f, -300.0f), glm::vec3(105.0f, -195.0f, -295.0f)), 93, glm::scale(glm::mat4(1.0f), glm::vec3(1.7f, 1.7f, 1.7f))));
    sceneObjects.emplace_back(std::make_shared<Object>(144, Spatium::AABB(glm::vec3(-1000.0f, -1100.0f, -1200.0f), glm::vec3(-995.0f, -1095.0f, -1195.0f)), 95, glm::rotate(glm::mat4(1.0f), glm::radians(225.0f), glm::vec3(0.0f, 1.0f, 0.0f))));
    sceneObjects.emplace_back(std::make_shared<Object>(145, Spatium::AABB(glm::vec3(-300.0f, -200.0f, 400.0f), glm::vec3(-295.0f, -195.0f, 405.0f)), 98, glm::translate(glm::mat4(1.0f), glm::vec3(0.0f, 0.0f, 30.0f))));
}