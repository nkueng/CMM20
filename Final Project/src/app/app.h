#pragma once

#include "gui/imgui_multiplot.h"
#include <gui/application.h>
#include <gui/shader.h>
#include <gui/renderer.h>
#include <gui/light.h>
#include <gui/camera.h>
#include <gui/ImGuizmo.h>
#include <utils/logger.h>
#include <robot/Robot.h>
#include <robot/RobotControl.h>
#include <robot/MotionGenerator.h>
#include <robot/BaseTrajectoryGenerator.h>
#include <ode/ODERBEngine.h>

/**
    Space: run/pause sim
    Click on robot: highlight the body part, print out the name.
*/

class App : public Basic3DAppWithShadows {
public:
    App(const char* title = "CRL Playground - Creature Locomotion app", std::string iconPath = CRL_DATA_FOLDER"/icons/icon.png")
        : Basic3DAppWithShadows(title, iconPath) {

        camera = TrackingCamera(5);
        camera.aspectRatio = float(width) / height;
        camera.rotAboutUpAxis = -0.75;
        camera.rotAboutRightAxis = 0.5;

        light.s = 0.03f;
        shadowbias = 0.0f;

        glEnable(GL_DEPTH_TEST);

        showConsole = true;
        automanageConsole = true;
        Logger::maxConsoleLineCount = 10;
        consoleHeight = 225;

        odeRbEngine->loadRBsFromFile(CRL_DATA_FOLDER "/environment/Ground.rbs");

        robot.showMeshes = false;
        robot.showSkeleton = true;

        robot.setRootState(P3D(0, 0.5, 0), Quaternion::Identity());

        // set all the joints to position mode
        for (int i = 0; i < robot.getJointCount(); i++) {
            robot.getJoint(i)->controlMode = RBJointControlMode::POSITION_MODE;
        }

        this->targetFramerate = 30;
        this->limitFramerate = true;
		autonomousNavigationOn = false;

        for(int i = 0; i < MY_PLOT_N; i++)
            myPlotValues[i] = 0;

        // some couts for debugging
        // dVector q;
        // genCoords.getQ(q);
        // cout << "generalized coords q: " << q << endl;
		for (int i = 0; i < 6; i++)
		{
			footEnabled[i] = true;
		}
		brokenFootChoice = 0;
		gaitChoice = 0;
        
    }

    virtual ~App() override {
    }

    virtual void resizeWindow(int width, int height) override {
        camera.aspectRatio = float(width) / height;
        return Application::resizeWindow(width, height);
    }

    bool mouseMove(double xpos, double ypos) override {
        P3D rayOrigin;
        V3D rayDirection;
        camera.getRayFromScreenCoordinates(xpos, ypos, rayOrigin, rayDirection);
        Ray mouseRay(rayOrigin, rayDirection);

        if (mouseState.dragging == false) {
            // this bit of code will check for mouse-ray robot intersections all the time.
            if (selectedRB != NULL)
                selectedRB->rbProps.selected = false;

            P3D selectedPoint;

            selectedRB = robot.getFirstRBHitByRay(mouseRay, selectedPoint, false, true);

            if (selectedRB != NULL) {
                selectedRB->rbProps.selected = true;
            }
        }
        else {
        }

        camera.processMouseMove(mouseState, keyboardState);
        return true;
    }

    bool mouseButtonReleased(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (selectedRB) {
                selectedRB->rbProps.selected = false;
                selectedRB = nullptr;
            }
        }
        return true;
    }

    bool mouseButtonPressed(int button, int mods) override {
        if (button == GLFW_MOUSE_BUTTON_LEFT || button == GLFW_MOUSE_BUTTON_RIGHT) {
            if (selectedRB != NULL) {
                if (selectedRB->pJoint == NULL)
                    Logger::consolePrint("Clicked on BodyLink %s (root)\n", selectedRB->name.c_str());
                else
                    Logger::consolePrint("Clicked on BodyLink %s (parent joint is %s with id %d))\n", selectedRB->name.c_str(), selectedRB->pJoint->name.c_str(), selectedRB->pJoint->jIndex);
            }
        }

        return true;
    }

    bool scrollWheel(double xoffset, double yoffset) override {
        camera.processMouseScroll(xoffset, yoffset);
        return true;
    }

    void computeAndApplyControlInputs(double dt) {
        simTime += dt;

        // check if default pose has been reached
        if (simTime >= timeToDefPose + .1)
        {
			dVector targetBasePosition = dVector::Zero(2); //= 2. * dVector::Ones(2);
			targetBasePosition(0) = guizmoPosition(0); // = dVector(double(guizmoPosition(0)), guizmoPosition(2));
			targetBasePosition(1) = guizmoPosition(2);
            double targetBaseYaw = PI / 2.;
			if (autonomousNavigationOn)
			{
				BaseTrajectoryGenerator::BaseTrajectoryGeneratorMessage baseTrajectoryMsg;
				baseTrajectoryMsg = baseTrajectoryGenerator.generateBaseTrajectory(targetBasePosition, targetBaseYaw);
				desiredZVelocity = baseTrajectoryMsg.referenceBaseZVelocity;
				desiredTurningRate = baseTrajectoryMsg.referenceYawRate;
				desiredXVelocity = 0.;
				if (baseTrajectoryMsg.trajectoryHasComeToEnd)
				{
					//gaitControl = 0;
					std::cout << "point is reached" << std::endl;
					autonomousNavigationOn = 0;
				}
			}
            if (gaitControl) {
                controller.controlMode = RobotControlMode::GAIT;
                MotionGenerator::MotionGeneratorMessage msg;
                msg = motionGenerator.generateMotion(desiredZVelocity, desiredXVelocity, desiredTurningRate, &gaitMode[0][0], &footEnabled[0]);
                // DEBUG
                // std::cout << "Motion generated." << std::endl;
                // interface for gait controller
                gait_des_pos.col(0) = V3D(msg.desiredBasePosition);
                gait_des_vel.col(0) = V3D(msg.desiredBaseVelocity);//dVector::Zero(3); 
                gait_des_acc.col(0) = V3D();
                gait_des_pos.col(1) = V3D(msg.desiredBaseOrient);
                gait_des_vel.col(1) = V3D(msg.desiredTurningRate);
                // DEBUG
                // std::cout << "Base motion set." << std::endl;
                feet_on_ground = dVector::Zero(6);
                // DEBUG
                // std::cout << "Number of foot desired positions = " << msg.desiredFootPositions.size() << std::endl;
                for(int i = 2; i < (msg.desiredFootPositions.size() + 2); i++) {
                    // DEBUG
                    // std::cout << "i = " << i << std::endl;
					if ((i - 2 + 1) == brokenFootChoice)
					{
						feet_on_ground(i - 2) = false;
						gait_des_pos.col(i) = V3D(controller.footRFController.getWorldPos());
						gait_des_vel.col(i) = V3D(controller.footRFController.getLinVel());
						gait_des_acc.col(i) = V3D(0.,0.,0.);
					}
					else
					{
						feet_on_ground(i - 2) = msg.isInContact[i - 2];
						// DEBUG
						// std::cout << "Feet on ground set." << std::endl;
						gait_des_pos.col(i) = V3D(msg.desiredFootPositions[i - 2]);
						// std::cout << "Feet position set." << std::endl;
						gait_des_vel.col(i) = V3D(msg.desiredFootVelocities[i - 2]);
						// std::cout << "Feet velocities set." << std::endl;
						gait_des_acc.col(i) = V3D(msg.desiredFootAccelerations[i - 2]);
					}

                }
                controller.gait_des_pos = gait_des_pos;
                controller.gait_des_vel = gait_des_vel;
                controller.gait_des_acc = gait_des_acc;
                controller.feet_on_ground = feet_on_ground;
                // TODO2, TODO4: Pass here message to controller members.
                // DEBUG
                // std::cout << "Controller member set." << std::endl;
            }
            else {
                // compute required torques to reach reference position from GUI sliders
                feet_on_ground = dVector::Ones(6);
                feet_on_ground(0) = RF_on_ground;
                feet_on_ground(1) = LF_on_ground;
                feet_on_ground(2) = LM_on_ground;
                feet_on_ground(3) = RM_on_ground;
                feet_on_ground(4) = RH_on_ground;
                feet_on_ground(5) = LH_on_ground;

                Matrix ref_offsets(3, 8);
                ref_offsets << base_pos_ref_offset,
                               base_rot_ref_offset,
                               footRF_ref_offset,
                               footLF_ref_offset,
                               footLM_ref_offset,
                               footRM_ref_offset,
                               footRH_ref_offset,
                               footLH_ref_offset;
                // std::cout << "The matrix ref_offsets is of size " << ref_offsets.rows() << "x" << ref_offsets.cols() << std::endl;
                // std::cout << ref_offsets << std::endl;
                controller.slider_ref_offsets = ref_offsets;
                controller.feet_on_ground = feet_on_ground;
				//std::cout << "guizmoposition: " << guizmoPosition(0) << std::endl;
            }

            // compute torque commands based on desired accelerations/offsets
            dVector tau = controller.getControlTorque();

            // DEBUG
            // P3D robotBasePos = controller.genCoords.getWorldCoordinates(P3D(0., 0., 0.), robot.root);
            // std::cout << "world coords of base: " << robotBasePos.x << ", " << robotBasePos.y << ", " << robotBasePos.z << std::endl;

            // employ force control 
            int i = 0;
            for(const auto joint : robot.jointList) {
                joint->controlMode = RBJointControlMode::FORCE_MODE; 
                joint->desiredControlSignal = tau[i];
                i++;
            }
        }
        else 
        {
            // let's say we want the robot to go from the zero angle configuration
            // (as it is loaded) to the default pose in timeToDefPose seconds:
            double interpVal = simTime / timeToDefPose;
            boundToRange(&interpVal, 0, 1.0);

            for(const auto joint : robot.jointList) {

                joint->desiredControlSignal = interpVal * joint->defaultJointAngle;  
            }
        }
        
    }

    void process() override {
        if (appIsRunning == false)
            return;

        // we need to do enough work here until the simulation time is caught up
        // with the display time...
        double tmpT = 0;
        while (tmpT < 1.0 / targetFramerate) {
            tmpT += dt;
            computeAndApplyControlInputs(dt);

            odeRbEngine->step(dt);

        }

        light.target.x() = robot.root->state.pos.x;
        light.target.z() = robot.root->state.pos.z;

        camera.target.x = robot.root->state.pos.x;
        camera.target.z = robot.root->state.pos.z;
    }

    virtual void drawAuxiliaryInfo() {
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        drawFPS();

        drawConsole();

        drawImGui();

        ImGui::EndFrame();
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    }

    // objects drawn with a shadowMapRenderer (during shadow pass) will cast a shadow
    virtual void drawShadowCastingObjects() override {
        robot.draw(shadowMapRenderer);
    }

    // objects drawn with a shadowShader (during the render pass) will have shadows cast on them
    virtual void drawObjectsWithShadows() override {
        ground.draw(shadowShader, V3D(0.6, 0.6, 0.8));
        if (DRAW_ORIGIN)
        {
            drawArrow3d(P3D(0., 0., 0.), V3D(.1, 0, 0), 0.01, shadowShader, V3D(1, 0, 0));
            drawArrow3d(P3D(0., 0., 0.), V3D(0, .1, 0), 0.01, shadowShader, V3D(0, 1, 0));
            drawArrow3d(P3D(0., 0., 0.), V3D(0, 0, .1), 0.01, shadowShader, V3D(0, 0, 1));
        }
        
        if (DRAW_REF_YAWS)
        {
            // draw estimated yaw angle of MotionGenerator
            P3D base_pos = controller.baseController.getWorldPos();
            V3D direction_1 = V3D(.2 * sin(motionGenerator.desiredYawAtTimeStamp), 0., .2 * cos(motionGenerator.desiredYawAtTimeStamp));
            V3D direction_2 = V3D(.2 * sin(motionGenerator.desiredYaw), 0., .2 * cos(motionGenerator.desiredYaw));
            drawArrow3d(base_pos, direction_1, 0.01, shadowShader, V3D(1, 0, 0));
            drawArrow3d(base_pos, direction_2, 0.01, shadowShader, V3D(0, 1, 0));
        }
        

        if (DRAW_STABILITY_REGION)
        {
            // draw lines between feet on ground
            Matrix feetPos(3, 6);
            feetPos << V3D(controller.footRFController.getWorldPos()),
                       V3D(controller.footLFController.getWorldPos()),
                       V3D(controller.footLMController.getWorldPos()),
                       V3D(controller.footRMController.getWorldPos()),
                       V3D(controller.footRHController.getWorldPos()),
                       V3D(controller.footLHController.getWorldPos());
            for (int i = 0; i < 5; ++i)
            {
                // std::cout << "i: " << i << std::endl;
                if (feet_on_ground(i))
                {
                    for (int j = i + 1; j < 6; ++j)
                    {
                        // std::cout << "j: " << j << std::endl;
                        if (feet_on_ground(j))
                        {
                            drawCylinder(getP3D(feetPos.col(i)), getP3D(feetPos.col(j)), 0.01, shadowShader, V3D(1, 0, 0));
                        }
                    }
                }
            }

            // project root pos onto ground
            P3D basePos = controller.baseController.getWorldPos();
            basePos[1] = 0.;
            drawSphere(basePos, .02, shadowShader, V3D(0, 1, 0));
        }

        if (DRAW_TARGET_FOOTHOLDS)
        {
            // draw target foothold positions
            if (!motionGenerator.targetFootholdPositions.empty())
            {
                for (int i = 0; i < motionGenerator.targetFootholdPositions.size(); ++i)
                {
                    drawSphere(motionGenerator.targetFootholdPositions[i], 0.02, shadowShader, V3D(0, 0, 1));
                }
            }
        }
    }

    // objects drawn with basic shadowShader (during the render pass) will not have shadows cast on them
    virtual void drawObjectsWithoutShadows() override {
        robot.draw(basicShader);
    }

    virtual bool keyPressed(int key, int mods) override {
        if (key == GLFW_KEY_SPACE) {
            appIsRunning = !appIsRunning;
            return true;
        }
        return false;
    }

    // draws the UI using ImGui. you can add your own UI elements (see below)
    virtual void drawImGui(){

        using namespace ImGui;

        SetNextWindowPos(ImVec2(0, 0), ImGuiCond_Once);
        Begin("Main Menu", NULL, ImGuiWindowFlags_AlwaysAutoResize);

        Text("Play:"); SameLine(); ToggleButton("Play App", &appIsRunning);

        if (appIsRunning == false) {
            SameLine();
            if (ArrowButton("tmp", ImGuiDir_Right)) {
                appIsRunning = true;
                process();
                appIsRunning = false;
            }
        }

        Checkbox("Draw Console", &showConsole);

        if (TreeNode("Draw options...")) {
            Checkbox("Draw Reference Yaws", &DRAW_REF_YAWS);
            Checkbox("Draw Origin", &DRAW_ORIGIN);
            Checkbox("Draw Target Footholds", &DRAW_TARGET_FOOTHOLDS);
            Checkbox("Draw Stability Region", &DRAW_STABILITY_REGION);
            Checkbox("Draw Meshes", &robot.showMeshes);
            Checkbox("Draw Skeleton", &robot.showSkeleton);
            Checkbox("Draw Joint Axes", &robot.showJointAxes);
            Checkbox("Draw Joint Limits", &robot.showJointLimits);
            Checkbox("Draw Collision Primitives", &robot.showCollisionSpheres);
            Checkbox("Draw MOI box", &robot.showMOI);

            TreePop();
        }

        Text("Robot Control");
            Indent();
			// Text("Regularizer for IK solver: ");
			// SliderScalar("lambda", ImGuiDataType_Double, &controller.objFunc.log_c_reg, &controller.objFunc.log_c_reg_min, &controller.objFunc.log_c_reg_max);
            
			Text("Autonomous Navigation: "); SameLine(); ToggleButton("##1", &autonomousNavigationOn);
			if (autonomousNavigationOn)
			{
				gaitChoice = 0;
				gaitControl = true;
			}
			Text("Gait Control:"); SameLine(); ToggleButton("Gait Control", &gaitControl);
    			Indent();
                //Here the gait can be chosen by either clicking on the desired mode or selecting the gait pattern by hand
    			Text("Choose Gait Mode:");
    			const char * items[6] = { "tripod left first", "tripod right first", "metachronal", "lifting walk", "custom gait", "broken leg"};
    			Combo("##2", (int*)&gaitChoice, items, 5);
    			switch (gaitChoice)
    			{
    			// tripod left first
                case 0:
                    // desiredZVelocity = 0.025;
                    desiredXVelocity = 0.;
                    // desiredTurningRate = 0.;
                    maxZVelocity = 0.045;
                    maxTurningRate = 0.125;

    				for (int i = 0; i < 6; i++)
    				{
    					for (int j = 0; j < 6; j++)
    					{
    						if ((i + j) % 2 == 0)
    						{
    							gaitMode[i][j] = true;
    						}
    						else
    						{
    							gaitMode[i][j] = false;
    						}
    					}
    				}
    				break;
                //tripod right first
    			case 1:
                    // desiredZVelocity = 0.025;
                    desiredXVelocity = 0.;
                    // desiredTurningRate = 0.;
                    maxZVelocity = 0.045;
                    maxTurningRate = 0.125;

    				for (int i = 0; i < 6; i++)
    				{
    					for (int j = 0; j < 6; j++)
    					{
    						if ((i + j) % 2 == 1)
    						{
    							gaitMode[j][i] = true;
    						}
    						else
    						{
    							gaitMode[j][i] = false;
    						}
    					}
    				}
    				break;
                // metachromal
    			case 2:
                    // desiredZVelocity = 0.01;
                    desiredXVelocity = 0.;
                    // desiredTurningRate = 0.;
                    maxZVelocity = 0.0125;
                    maxTurningRate = 0.08;

    				for (int i = 0; i < 6; i++)
    				{
    					for (int j = 0; j < 6; j++)
    					{
    						if (i == j)
                            {
    							gaitMode[j][i] = false;
    						}
    						else
    						{
    							gaitMode[j][i] = true;
    						}
    					}
    				}
                    break;
    			// lifting walk
    			case 3:
    				desiredZVelocity = 0.005;
    				desiredXVelocity = 0.;
    				desiredTurningRate = 0.;
    				footEnabled[0] = false;
    				footEnabled[1] = false;
    				for (int i = 0; i < 6; i++)
    				{
    					for (int j = 0; j < 6; j++)
    					{
    						if ((i == 1 && j == 2) || (i == 2 && j == 3) || (i == 3 && j == 4) || (i == 4 && j == 5))
    						{
    							gaitMode[j][i] = true;
    						}
    						else if (j < 2)
    						{
    							gaitMode[j][i] = false;
    						}
    						else
    						{
    							gaitMode[j][i] = true;
    						}
    					}
    				}
    				break;
                // choose
                case 4:
                    desiredZVelocity = 0.015;
                    desiredXVelocity = 0.;
                    desiredTurningRate = 0.;
    				break;
    			//broken leg
    			case 5:
    				desiredZVelocity = 0.011;
    				desiredXVelocity = 0.;
    				desiredTurningRate = 0.;
    				footEnabled[brokenFootChoice-1] = false;

    				for (int i = 0; i < 6; i++)
    				{
    					for (int j = 0; j < 6; j++)
    					{
    						
    						if (j == (brokenFootChoice - 1))
    						{
    							gaitMode[j][i] = false;
    						}
    					}
    				}
    				break;
    			}

    			const char * item_broken[7] = { "None", "RF", "LF", "LM", "RM", "RB", "LB" };

                // display on GUI as 6x6 array
    			for (int j = 0; j < 6; j++)
    			{
    				// Text(item_broken[j+1]);
    				// SameLine();
    				for (int i = 0; i < 6; i++)
    				{
    					ToggleButton("", &gaitMode[j][i]);
    					if (i < 5)
    					{
    						SameLine();
    					}
    				}
                    SameLine();
                    Text(item_broken[j+1]);
    			}

    			Text("Which leg is injured?");
    			Combo("##3", (int*)&brokenFootChoice, item_broken, 7);
    			if (brokenFootChoice != 0)
    			{
    				gaitChoice = 5;
    			}

                Text("Gait Control:");
                SliderScalar("Forward Velocity", ImGuiDataType_Double, &desiredZVelocity, &kp_min, &maxZVelocity);
                SliderScalar("Turning Rate", ImGuiDataType_Double, &desiredTurningRate, &kp_min, &maxTurningRate);

                Text("Gait Controller Gains: ");
                SliderScalar("Stance Weight", ImGuiDataType_Double, &motionGenerator.stanceWeight, &weight_min, &weight_max);
                SliderScalar("Swing Weight", ImGuiDataType_Double, &motionGenerator.swingWeight, &weight_min, &weight_max);
                // SliderScalar("Contact Tolerance", ImGuiDataType_Double, &motionGenerator.contactTolerance, &weight_min, &weight_max);
                Unindent();


			Text("Slider Control:");
                Indent();
                // Text("Base Position PD Controller: ");
                // SliderScalar("kp_base", ImGuiDataType_Double, &controller.kp_base, &kp_min, &kp_max);
                // SliderScalar("kd_base", ImGuiDataType_Double, &controller.kd_base, &kd_min, &kd_max);

                Text("Base Position: ");
                SliderScalar("X", ImGuiDataType_Double, &base_pos_ref_offset[0], &x_min, &x_max);
                SliderScalar("Y", ImGuiDataType_Double, &base_pos_ref_offset[1], &y_min, &y_max);
                SliderScalar("Z", ImGuiDataType_Double, &base_pos_ref_offset[2], &z_min, &z_max);

                Text("Base Orientation: ");
                SliderScalar("Roll", ImGuiDataType_Double, &base_rot_ref_offset[2], &z_min, &z_max);
                SliderScalar("Pitch", ImGuiDataType_Double, &base_rot_ref_offset[0], &x_min, &x_max);
                SliderScalar("Yaw", ImGuiDataType_Double, &base_rot_ref_offset[1], &y_min, &y_max);

                // Text("Feet Position PD Controller: ");
                // SliderScalar("kp_feet", ImGuiDataType_Double, &controller.kp_feet, &kp_min, &kp_max);
                // SliderScalar("kd_feet", ImGuiDataType_Double, &controller.kd_feet, &kd_min, &kd_max);

                Text("Foot RF Position: "); SameLine(); Checkbox("RF On Ground", &RF_on_ground);
                SliderScalar("X_RF", ImGuiDataType_Double, &footRF_ref_offset[0], &x_min, &x_max);
                SliderScalar("Y_RF", ImGuiDataType_Double, &footRF_ref_offset[1], &y_min, &y_max);
                SliderScalar("Z_RF", ImGuiDataType_Double, &footRF_ref_offset[2], &z_min, &z_max);

                Text("Foot LF Position: "); SameLine(); Checkbox("LF On Ground", &LF_on_ground);
                SliderScalar("X_LF", ImGuiDataType_Double, &footLF_ref_offset[0], &x_min, &x_max);
                SliderScalar("Y_LF", ImGuiDataType_Double, &footLF_ref_offset[1], &y_min, &y_max);
                SliderScalar("Z_LF", ImGuiDataType_Double, &footLF_ref_offset[2], &z_min, &z_max);

                Text("Foot LM Position: "); SameLine(); Checkbox("LM On Ground", &LM_on_ground);
                SliderScalar("X_LM", ImGuiDataType_Double, &footLM_ref_offset[0], &x_min, &x_max);
                SliderScalar("Y_LM", ImGuiDataType_Double, &footLM_ref_offset[1], &y_min, &y_max);
                SliderScalar("Z_LM", ImGuiDataType_Double, &footLM_ref_offset[2], &z_min, &z_max);

                Text("Foot RM Position: "); SameLine(); Checkbox("RM On Ground", &RM_on_ground);
                SliderScalar("X_RM", ImGuiDataType_Double, &footRM_ref_offset[0], &x_min, &x_max);
                SliderScalar("Y_RM", ImGuiDataType_Double, &footRM_ref_offset[1], &y_min, &y_max);
                SliderScalar("Z_RM", ImGuiDataType_Double, &footRM_ref_offset[2], &z_min, &z_max);

                // Text("Foot RH Position: "); SameLine(); Checkbox("RH On Ground", &RH_on_ground);
                // SliderScalar("X_RH", ImGuiDataType_Double, &footRH_ref_offset[0], &x_min, &x_max);
                // SliderScalar("Y_RH", ImGuiDataType_Double, &footRH_ref_offset[1], &y_min, &y_max);
                // SliderScalar("Z_RH", ImGuiDataType_Double, &footRH_ref_offset[2], &z_min, &z_max);

                // Text("Foot LH Position: "); SameLine(); Checkbox("LH On Ground", &LH_on_ground);
                // SliderScalar("X_LH", ImGuiDataType_Double, &footLH_ref_offset[0], &x_min, &x_max);
                // SliderScalar("Y_LH", ImGuiDataType_Double, &footLH_ref_offset[1], &y_min, &y_max);
                // SliderScalar("Z_LH", ImGuiDataType_Double, &footLH_ref_offset[2], &z_min, &z_max); 
                Unindent();

            Text("guizmo:");
            Checkbox("enabled", &guizmoEnabled);
            InputScalarN("position", ImGuiDataType_Double, guizmoPosition.data(), 3);

            // ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
            Unindent();
        

        // example code on how to use the ImGuizmo
        if(guizmoEnabled)
        {
            ImGuizmo::BeginFrame();
            ImGuiIO& io = ImGui::GetIO();
            ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
            SliderScalar("Guizmo Yaw", ImGuiDataType_Double, &guizmoYaw, &kp_min, &kp_max);

            // we use it in translate mode and need to provide the corresponding
            // transformation matrix to ImGuizmo ...
            auto transform = glm::translate(glm::mat4(1.f), toGLM(guizmoPosition));
            ImGuizmo::Manipulate(glm::value_ptr(camera.getViewMatrix()), glm::value_ptr(camera.getProjectionMatrix()), ImGuizmo::TRANSLATE, ImGuizmo::WORLD, glm::value_ptr(transform));

            // ... and thus after manipulation, we extract the changed position
            // from the transformation matrix.
            guizmoPosition = Vector3d(
                transform[3][0],
                transform[3][1],
                transform[3][2]
                );
        }

        float window_h = GetWindowHeight();
        End();

        // DEBUG
  //       SetNextWindowPos(ImVec2(0, window_h), ImGuiCond_Always);
  //       Begin("Debugging", NULL, ImGuiWindowFlags_AlwaysAutoResize);
        
  //       // make a plot
  //       PlotLines("Des RF Vel y", RF_des_pos_y, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       PlotLines("Cur RF Vel y", RF_cur_pos_y, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       PlotLines("Des RF Pos y", LF_des_pos_y, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       PlotLines("Cur RF Pos y", LF_cur_pos_y, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200});

  //       // { // D:
  //       //     const char ** names = new const char*[1]; names[0] = "Des LF Pos y";
  //       //     ImColor PLOT_COLOR = ImColor(249, 38, 114);
  //       //     ImColor *colors = { &PLOT_COLOR };
  //       //     float **functionValues = new float*[2];
  //       //     functionValues[0] = LF_des_pos_y;
  //       //     // functionValues[1] = RF_des_pos_x;
  //       //     PlotMultiLines("Feet Pos y", 
  //       //                    1, 
  //       //                    names, 
  //       //                    colors, 
  //       //                    [](const void *data, int idx)->float { return ((const float*)data)[idx]; }, 
  //       //                    ((const void *const *)functionValues), 
  //       //                    MY_PLOT_N, 
  //       //                    MY_PLOT_N, 
  //       //                    0, 
  //       //                    0., 
  //       //                    *(std::max_element(std::begin(LF_des_pos_y), std::end(LF_des_pos_y))), 
  //       //                    ImVec2(0, 150));
  //       //     LF_des_pos_y[myPlotCounter] = gait_des_pos.col(3)[1];
  //       //     // RF_cur_pos_x[myPlotCounter] = controller.footRFController.getWorldPos()[0];
  //       // }

  //       // update plot values
  //       RF_des_pos_y[myPlotCounter] = gait_des_vel.col(2)[1];
  //       RF_cur_pos_y[myPlotCounter] = controller.footRFController.getLinVel()[1];
		// LF_des_pos_y[myPlotCounter] = gait_des_pos.col(2)[1];
		// LF_cur_pos_y[myPlotCounter] = controller.footRFController.getWorldPos()[1];
		
  //       //LF_des_pos_y[myPlotCounter] = gait_des_pos.col(1)[1];
  //       //LF_cur_pos_y[myPlotCounter] = controller.footLFController.getWorldPos()[1];


  //       // // make a plot
  //       // PlotLines("Des RF Pos x", RF_des_pos_x, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       // PlotLines("Cur RF Pos x", RF_cur_pos_x, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       // PlotLines("Des LF Pos x", RF_des_pos_x, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200}); SameLine();
  //       // PlotLines("Cur LF Pos x", RF_cur_pos_x, MY_PLOT_N, myPlotCounter, NULL, FLT_MAX, FLT_MAX, {200, 200});

  //       // // update plot values
  //       // RF_des_pos_x[myPlotCounter] = gait_des_pos.col(1)[0];
  //       // RF_cur_pos_x[myPlotCounter] = controller.footRFController.getWorldPos()[0];
  //       // LF_des_pos_x[myPlotCounter] = gait_des_pos.col(2)[0];
  //       // LF_cur_pos_x[myPlotCounter] = controller.footLFController.getWorldPos()[0];

  //       myPlotCounter = (myPlotCounter+1) % MY_PLOT_N;

  //       End();

    }

    virtual bool drop(int count, const char** fileNames) override {
        return true;
    }

public:
    SimpleGroundModel ground;   // model to draw the ground

    // the simulation engine
    crl::sim::ODERBEngine* odeRbEngine = new crl::sim::ODERBEngine();

    // the robot to load. uncomment/comment to change robot model
    Robot robot =
        Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/hex.rbs");
//      Robot(odeRbEngine, CRL_DATA_FOLDER"/robots/simple/dog.rbs");

    // object for control
    RobotControl controller = RobotControl(&robot);

    Matrix gait_des_pos = Matrix::Zero(3, 8); 
    Matrix gait_des_vel = Matrix::Zero(3, 8);
    Matrix gait_des_acc = Matrix::Zero(3, 8);

    // (default) references from GUI for controller
    bool gaitControl = true;

    V3D base_pos_ref_offset = V3D(0., 0., 0.);
    V3D base_rot_ref_offset = V3D(0., 0., 0.);

    V3D footRF_ref_offset = V3D(0., 0., 0.);
    bool RF_on_ground = true;
    V3D footLF_ref_offset = V3D(0., 0., 0.);
    bool LF_on_ground = true;
    V3D footLM_ref_offset = V3D(0., 0., 0.);
    bool LM_on_ground = true;
    V3D footRM_ref_offset = V3D(0., 0., 0.);
    bool RM_on_ground = true;
    V3D footRH_ref_offset = V3D(0., 0., 0.);
    bool RH_on_ground = true;
    V3D footLH_ref_offset = V3D(0., 0., 0.);
    bool LH_on_ground = true;

    // slider limits
    double x_min = -.1; double x_max = .1;
    double y_min = -.1; double y_max = .1;
    double z_min = -.1; double z_max = .1;
    double kp_min = 0.; double kp_max = 100.;
    double kd_min = 0.; double kd_max = 10.;
    double weight_min = 0.; double weight_max = 1.;

    RobotRB* selectedRB = NULL; // robot rigid body selected by mouse, = NULL when nothing selected
    bool appIsRunning = false;

    double dt = 1 / 120.0;      // time step for the simulation
    double simTime = 0.0;       // current simulation time
    double timeToDefPose = 1.0; // time to get to pose with default joint angles

    // object for motion generator
    MotionGenerator::MotionGenerator motionGenerator = MotionGenerator::MotionGenerator(&robot, dt);
    BaseTrajectoryGenerator::BaseTrajectoryGenerator baseTrajectoryGenerator = BaseTrajectoryGenerator::BaseTrajectoryGenerator(&robot);

    // example variables for how to use ImGui
    bool myBool = true;
    double myDouble = 1;
    double myDouble2 = 1.;
    Vector3d myVector3d = {1,2,3};
    int myPlotCounter = 0;
    const static int MY_PLOT_N = 100;
    float myPlotValues[MY_PLOT_N];

    // example for a 3d guizmo
    bool guizmoEnabled = false;
    Vector3d guizmoPosition = Vector3d{0,0,0};
    double guizmoYaw = 0.;

    // plotting
    float RF_des_pos_y[MY_PLOT_N]; 
    float RF_cur_pos_y[MY_PLOT_N];
    float LF_des_pos_y[MY_PLOT_N]; 
    float LF_cur_pos_y[MY_PLOT_N];
    // float RF_des_pos_x[MY_PLOT_N]; 
    // float RF_cur_pos_x[MY_PLOT_N];
    // float LF_des_pos_x[MY_PLOT_N]; 
    // float LF_cur_pos_x[MY_PLOT_N];

	int gaitChoice;
	int brokenFootChoice;
	bool gaitMode[6][6];
	bool footEnabled[6];
    double desiredZVelocity = 0.025;
    double desiredXVelocity = 0.0;
    double desiredTurningRate = 0.0;
    double maxZVelocity;
    double maxTurningRate;
	bool autonomousNavigationOn;
    dVector feet_on_ground = dVector::Ones(6);
    bool DRAW_ORIGIN = false;
    bool DRAW_REF_YAWS = false;
    bool DRAW_STABILITY_REGION = false;
    bool DRAW_TARGET_FOOTHOLDS = false;
};
