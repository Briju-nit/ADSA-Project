#include <iostream>
#include <vector>
#include <cmath>
#include <limits>

using namespace std;

struct Point {
    double x, y, z;
};

struct Node {
    Point point;
    Node* left;
    Node* right;
};

Node* createNode(const Point& point) {
    Node* newNode = new Node;
    newNode->point = point;
    newNode->left = nullptr;
    newNode->right = nullptr;
    return newNode;
}

Node* insertNode(Node* root, const Point& point, int depth) {
    if (root == nullptr) {
        return createNode(point);
    }

    int currentDim = depth % 3;

    if (currentDim == 0) {
        if (point.x < root->point.x) {
            root->left = insertNode(root->left, point, depth + 1);
        } else {
            root->right = insertNode(root->right, point, depth + 1);
        }
    } else if (currentDim == 1) {
        if (point.y < root->point.y) {
            root->left = insertNode(root->left, point, depth + 1);
        } else {
            root->right = insertNode(root->right, point, depth + 1);
        }
    } else {
        if (point.z < root->point.z) {
            root->left = insertNode(root->left, point, depth + 1);
        } else {
            root->right = insertNode(root->right, point, depth + 1);
        }
    }

    return root;
}

double calculateDistance(const Point& p1, const Point& p2) {
    double dx = p1.x - p2.x;
    double dy = p1.y - p2.y;
    double dz = p1.z - p2.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

double performCollisionDetection(Node* root, const Point& modelPoint, double searchRadius, int depth, bool& collisionDetected, double& maxPenetration, double& totalPenetration, int& count) {
    double penetrationDistance = -1.0;

    if (root == nullptr) {
        return penetrationDistance;
    }

    int currentDim = depth % 3;

    double distance = calculateDistance(root->point, modelPoint);

    if (distance <= searchRadius) {
        cout << "Colliding point: (" << root->point.x << ", " << root->point.y << ", " << root->point.z << ")" << endl;
        penetrationDistance = searchRadius - distance;
        cout << "Penetration distance: " << penetrationDistance << endl;
        maxPenetration = max(maxPenetration, penetrationDistance);
        totalPenetration += penetrationDistance;
        count++;
        collisionDetected = true;
    }

    if ((currentDim == 0 ? modelPoint.x - searchRadius <= root->point.x : currentDim == 1 ? modelPoint.y - searchRadius <= root->point.y
                                                                                             : modelPoint.z - searchRadius <= root->point.z)) {
        double leftPenetrationDistance = performCollisionDetection(root->left, modelPoint, searchRadius, depth + 1, collisionDetected, maxPenetration, totalPenetration, count);
        if (leftPenetrationDistance > 0) {
            penetrationDistance = (penetrationDistance < 0) ? leftPenetrationDistance : min(penetrationDistance, leftPenetrationDistance);
        }
    }

    if ((currentDim == 0 ? modelPoint.x + searchRadius >= root->point.x : currentDim == 1 ? modelPoint.y + searchRadius >= root->point.y
                                                                                             : modelPoint.z + searchRadius >= root->point.z)) {
        double rightPenetrationDistance = performCollisionDetection(root->right, modelPoint, searchRadius, depth + 1, collisionDetected, maxPenetration, totalPenetration, count);
        if (rightPenetrationDistance > 0) {
            penetrationDistance = (penetrationDistance < 0) ? rightPenetrationDistance : min(penetrationDistance, rightPenetrationDistance);
        }
    }

    return penetrationDistance;
}

int main() {
    int numPoints;
    cout << "Enter the number of points in the point cloud: ";
    cin >> numPoints;

    vector<Point> points;
    points.reserve(numPoints);

    for (int i = 0; i < numPoints; ++i) {
        Point point;
        cout << "Enter coordinates for point " << i + 1 << " (x y z): ";
        cin >> point.x >> point.y >> point.z;
        points.push_back(point);
    }

    Node* root = nullptr;
    for (const auto& point : points) {
        root = insertNode(root, point, 0);
    }

    int numModelPoints;
    cout << "Enter the number of model points: ";
    cin >> numModelPoints;

    double maxPenetration = -1.0;
    double totalPenetration = 0.0;
    int totalCollisions = 0;

    for (int i = 0; i < numModelPoints; ++i) {
        Point modelPoint;
        cout << "Enter model point coordinates " << i + 1 << " (x y z): ";
        cin >> modelPoint.x >> modelPoint.y >> modelPoint.z;

        double searchRadius;
        cout << "Enter search radius for model point " << i + 1 << ": ";
        cin >> searchRadius;

        bool collisionDetected = false;

        double penetrationDistance = performCollisionDetection(root, modelPoint, searchRadius, 0, collisionDetected, maxPenetration, totalPenetration, totalCollisions);
        if (!collisionDetected) {
            cout << "No collision detected within the search radius." << endl;
        } else if (penetrationDistance > 0) {
            cout << "Minimum penetration distance: " << penetrationDistance << endl;
        }
    }

    double averagePenetration = (totalCollisions > 0) ? (totalPenetration / totalCollisions) : 0.0;

    cout << "Maximum penetration distance: " << maxPenetration << endl;
    cout << "Average penetration distance: " << averagePenetration << endl;

    return 0;
}
