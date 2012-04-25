#include "mex.h"
#include "string.h"
#include "Tools/Storage/Blackboard.h"
#include "Tools/Kinematics/KinematicMatrix.h"
#include "Tools/Kinematics/InverseKinematics.h"
#include "Tools/Math/RotationMatrix.h"
#include "Modules/DcmEngine.h"
#include "tuhh.h"
#include "../../src/sample.h"


void
mexFunction(int nlhs,mxArray *plhs[],int nrhs,const mxArray *prhs[])
{
    // get given method name
    char *method = mxArrayToString(prhs[0]);
    
    
    /*---------------------------------------
     *---------------------------------------
     *          own mex 
     *---------------------------------------
     *---------------------------------------*/
	
	if (!strcmp(method, "S_goDown")) {
		int time = (int) mxGetScalar(prhs[2]);
		float height = (float) mxGetScalar(prhs[1]);
		Sample::goDown(height, time);
	}
   
    
     /*---------------------------------------
     *---------------------------------------
     *          sdk mex 
     *---------------------------------------
     *---------------------------------------*/
    
    /*-------------------------------------
     * init tuhhSDK
     *-------------------------------------*/
    else if (!strcmp(method, "TU_init"))
    {
        TUHH::init();
    } 
    
    
    /*-------------------------------------
     *  setting time
     *-------------------------------------*/
    if (!strcmp(method, "BB_updateTime"))
    {
        int time = (int) mxGetScalar(prhs[1]);
        Blackboard::updateTime(time);
    }   
    
    /*-------------------------------------
     *  updating Center of Mass
     *-------------------------------------*/
    else if(!strcmp(method, "BB_updateCom"))
    {
        Blackboard::updateCom();
    }
    
    
    /*-------------------------------------
     *  reading Center of Mass
     *-------------------------------------*/
    else if(!strcmp(method, "BB_getCom"))
    {
        Vector3<float> com = Blackboard::getCom();
        plhs[0] = mxCreateDoubleMatrix(1,3,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        outPtr[0] = com.x;
        outPtr[1] = com.y;
        outPtr[2] = com.z;        
    }
    
    /*-------------------------------------
     *  setting Accelerometer Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateAccelerometerData"))
    {
        
        double *data = mxGetPr(prhs[1]);
        Vector3<float> accData(-(float)data[0],-(float)data[1],-(float)data[2]);
        
        Blackboard::updateAccelerometerData(accData);
    } 
    
    /*-------------------------------------
     *  reading Accelerometer Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getAccelerometerData"))
    {
        Vector3<float> accData = Blackboard::getAccelerometerData();
        plhs[0] = mxCreateDoubleMatrix(1,3,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        outPtr[0] = accData.x;
        outPtr[1] = accData.y;
        outPtr[2] = accData.z;        
    }
    
    /*-------------------------------------
     *  reading filtered Accelerometer Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getFAccelerometerData"))
    {
        Vector3<float> accData = Blackboard::getFilteredAccelerometerData();
        plhs[0] = mxCreateDoubleMatrix(1,3,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        outPtr[0] = accData.x;
        outPtr[1] = accData.y;
        outPtr[2] = accData.z;        
    }
    
    /*-------------------------------------
     *  setting Gyroscope Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateGyroscopeData"))
    {
        
        double *data = mxGetPr(prhs[1]);
        Vector2<float> gyrData((float)data[0],(float)data[1]);        
        Blackboard::updateGyroscopeData(gyrData);
    } 
    
    /*-------------------------------------
     *  reading Gyroscope Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getGyroscopeData"))
    {
        Vector2<float> gyrData = Blackboard::getGyroscopeData();
        plhs[0] = mxCreateDoubleMatrix(1,2,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        outPtr[0] = gyrData.x;
        outPtr[1] = gyrData.y;
             
    }
    
    /*-------------------------------------
     *  reading Angle Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getAngleData"))
    {
        Vector2<float> angleData = Blackboard::getAngleData();
        plhs[0] = mxCreateDoubleMatrix(1,2,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        outPtr[0] = angleData.x;
        outPtr[1] = angleData.y;              
    }
    
    /*-------------------------------------
     *  setting Joint Angle Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateJointAngles"))
    {
        vector<float> angleData;
        double *data = mxGetPr(prhs[1]);
        for (int i = 0; i < 26; i++)
            angleData.push_back((float)data[i]);        
        
        Blackboard::updateJointAngles(angleData);
    }
    
    /*-------------------------------------
     *  reading Joint Angle Data
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getJointAngles"))
    {
        char *chain = mxArrayToString(prhs[1]);
        vector<float> jointData = Blackboard::getJointAngles(chain);
        plhs[0] = mxCreateDoubleMatrix(1,jointData.size(),mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i< (int) jointData.size(); i++)
            outPtr[i] = jointData.at(i);                      
    }
    
    
    
     /*-------------------------------------
     *  updating Kinematic Matrices
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateKinematicMatrices"))
    {
        Blackboard::updateKinematicMatrices();
    }
    
    /*-------------------------------------
     *  reading Head Yaw
     *-------------------------------------*/
    else if (!strcmp(method, "BB_getKinematicMatrix"))
    {
        int index = (int) mxGetScalar(prhs[1]);
        KinematicMatrix matrix = Blackboard::getKinematicMatrix(index);
        plhs[0] = mxCreateDoubleMatrix(4,4,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i< 3; i++)
        {
            outPtr[4*i] = matrix.rotM.c[i].x;
            outPtr[4*i+1] = matrix.rotM.c[i].y;
            outPtr[4*i+2] = matrix.rotM.c[i].z;
            outPtr[4*i+3] = 0;
        }
        outPtr[12] = matrix.posV.x;
        outPtr[13] = matrix.posV.y;
        outPtr[14] = matrix.posV.z;
        outPtr[15] = 1;           
    }
    
    /*-------------------------------------
     *  update FSRs
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateFsr"))
    {
        vector<float> fsrData;
        double *data = mxGetPr(prhs[1]);
        for (int i = 0; i < 8; i++)
            fsrData.push_back((float)data[i]);        
        
        Blackboard::updateFsr(fsrData);
    }
    
    
    /*-------------------------------------
     *  updating angle estimation
     *-------------------------------------*/
    else if (!strcmp(method, "BB_updateAngleEstimation"))
    {
       Blackboard::updateAngleEstimation();      
    }    
    
       
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++  Inverse Kinematics  ++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    
    
    /*-------------------------------------
     * calculation of left leg angles
     *-------------------------------------*/
    if (!strcmp(method, "IK_getLLegAngles"))
    {
        double *inPtr = mxGetPr(prhs[1]);
        
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getLLegAngles(desired);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i);
        
    }
    
    /*-------------------------------------
     * calculation of right leg angles
     *-------------------------------------*/
    if (!strcmp(method, "IK_getRLegAngles"))
    {
       double *inPtr = mxGetPr(prhs[1]);
        
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getRLegAngles(desired);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i); 
    }
    
    
    /*-------------------------------------
     * calculation of left leg angles
     * with fixed HipYawPitch
     *-------------------------------------*/
    if (!strcmp(method, "IK_getFixedLLegAngles"))
    {
       double *inPtr = mxGetPr(prhs[1]);
       float hyp = (float) mxGetScalar(prhs[2]);
        
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getFixedLLegAngles(desired, hyp);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i); 
    }
    
    /*-------------------------------------
     * calculation of right leg angles
     * with fixed HipYawPitch
     *-------------------------------------*/
    if (!strcmp(method, "IK_getFixedRLegAngles"))
    {
       double *inPtr = mxGetPr(prhs[1]);
       float hyp = (float) mxGetScalar(prhs[2]);
        
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getFixedRLegAngles(desired, hyp);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i); 
    }
    
    /*-------------------------------------
     * calculation of left arm angles
     *-------------------------------------*/
    if (!strcmp(method, "IK_getLArmAngles"))
    {
       double *inPtr = mxGetPr(prhs[1]);
               
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getLArmAngles(desired);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i); 
    }
    
    
    /*-------------------------------------
     * calculation of right arm angles
     *-------------------------------------*/
    if (!strcmp(method, "IK_getRArmAngles"))
    {
       double *inPtr = mxGetPr(prhs[1]);
               
        RotationMatrix rotM =  RotationMatrix(
                (float) inPtr[0], (float) inPtr[4], (float) inPtr[8], 
                (float) inPtr[1], (float) inPtr[5], (float) inPtr[9],
                (float) inPtr[2], (float) inPtr[6], (float) inPtr[10]);
        
        Vector3<float> posV = Vector3<float>( (float) inPtr[12], 
                                              (float) inPtr[13],
                                              (float) inPtr[14]);
        
        KinematicMatrix desired = KinematicMatrix(rotM, posV);
        
        
        vector<float> angles = InverseKinematics::getRArmAngles(desired);
        
        plhs[0] = mxCreateDoubleMatrix(6,1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for (int i = 0; i < 6; i++)
            outPtr[i] = angles.at(i); 
    }
    

    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++         DCM          ++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++/*
    /*+++++++++++++++++++++++++++++++++++++*/
    
    /*-------------------------------------
     * setting DCM Time
     *-------------------------------------*/
    if (!strcmp(method, "DCM_setTime"))
    {
       int time = (int) mxGetScalar(prhs[1]);
       
       DcmEngine::setTime(time);       
    }
    
    /*-------------------------------------
     * setting DCM Time
     *-------------------------------------*/
    if (!strcmp(method, "DCM_init"))
    {
       DcmEngine::initialize(vector<float>(26));       
    }
    
    /*-------------------------------------
     * update commands
     *-------------------------------------*/
   
    if (!strcmp(method, "DCM_updateCommands"))
    {       
        vector<float> commands = DcmEngine::updateCommands();
        plhs[0] = mxCreateDoubleMatrix((int) commands.size(),1,mxREAL);
        double *outPtr = mxGetPr(plhs[0]);
        for ( int i = 0; i < (int) commands.size(); i++)
            outPtr[i] = commands.at(i);
        
    }
    
     /*-------------------------------------
     * setALias
     *-------------------------------------*/
    else if (!strcmp(method, "DCM_setAlias"))
    {
        // getting fields and pointers
        mxArray *name   = mxGetField(prhs[1],0,"aliasName");
        mxArray *update = mxGetField(prhs[1],0,"update");
        mxArray *mode   = mxGetField(prhs[1],0,"aliasMode");
        mxArray *time   = mxGetField(prhs[1],0,"timeAlias");
        mxArray *comm   = mxGetField(prhs[1],0,"commandsAlias");
        
        double *timePtr = mxGetPr(time);
        double *commPtr = mxGetPr(comm);

        // setting up
        Command commands;
        commands.jointName = mxArrayToString(name);
        commands.aliasMode = mxArrayToString(mode);
        commands.update    = mxArrayToString(update);
        
        
        int N = mxGetN(comm);
        int M = mxGetM(comm);        
        
        
        for (int i = 0; i < N;i++)
        {
            commands.timeAlias.push_back(vector<int>());
            commands.commandsAlias.push_back(vector<float>());
            for (int j = 0; j < M;j++)
           {
               if (commands.aliasMode == "time-mixed")
                   commands.timeAlias.at(i).push_back((int) timePtr[M*i + j]);
               else if ( i == 0)
                   commands.time.push_back((int) timePtr[j]);
               
               commands.commandsAlias.at(i).push_back((float) commPtr[M*i + j]);
           }
        }
        
        // send
        DcmEngine::setAlias(commands);
    }    
}







