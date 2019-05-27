#include <iostream>
#include <string>
#include <vector>
#include <algorithm>

using namespace std;

/**
 * Auto-generated code below aims at helping you parse
 * the standard input according to the problem statement.
 **/

constexpr int maxThrust = 100;

constexpr int boostAngleThreshold = 15;
constexpr int boostDistanceThreshold = 1000;

int main()
{
    bool canBoost = true;

    // game loop
    while (1) {
        int x;
        int y;
        int nextCheckpointX; // x position of the next check point
        int nextCheckpointY; // y position of the next check point
        int nextCheckpointDist; // distance to the next checkpoint
        int nextCheckpointAngle; // angle between your pod orientation and the direction of the next checkpoint
        cin >> x >> y >> nextCheckpointX >> nextCheckpointY >> nextCheckpointDist >> nextCheckpointAngle; cin.ignore();
        int opponentX;
        int opponentY;
        cin >> opponentX >> opponentY; cin.ignore();

        // Write an action using cout. DON'T FORGET THE "<< endl"
        // To debug: cerr << "Debug messages..." << endl;


        // You have to output the target position
        // followed by the power (0 <= thrust <= 100)
        // i.e.: "x y thrust"
        cout << nextCheckpointX << " " << nextCheckpointY << " ";

        const int angle = abs(nextCheckpointAngle);
        if (angle > 90)
        {
            cout << 0;
        }
        else
        {
            const bool useBoost = canBoost
                                  && (angle < boostAngleThreshold)
                                  && (nextCheckpointDist > boostDistanceThreshold);
            if (useBoost)
            {
                cout << "BOOST";
                canBoost = false;
            }
            else
            {
                cout << maxThrust;
            }
        }

        cout << endl;
    }
}
