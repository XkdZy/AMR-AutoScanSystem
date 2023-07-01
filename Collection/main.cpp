#include "header.h"

bool exitSys = false;
bool exitChildThread = false;
int mouseClickRow;
int mouseClickCol;

char portChar[10] = "COM";

cv::Mat colorImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
cv::Mat depthImage(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);//640x480

//���ݵ�Ŀ¼
string dataPath = "./imgs/";
AstraCameraD2C* astraCameraD2C;

//double deviceLength = 299.0; // ����29.9cm
//double deviceWidth = 265.0; // ��26.5cm
//double plain2cam = 154.0; // ����ͷ������ƽ�棺15.4cm
//double remainCam2Center = 20.0; // Ԥ������ͷ���浽����
// Բ����39cm���ڲࣩ*27cm*6cm---��r=40.525
//vector<Eigen::Vector3d> vScanTimers;
//vector<vector<Eigen::Vector3d>> vvBoxSegmentPos; // �ָ���ο��������λ��
//vector<std::shared_ptr<open3d::geometry::PointCloud>> vCropedCloud; // ������вü����ĵ���
//vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient;
//vector<Eigen::Vector3d> vCamPosition; // �������ɨ���λ��
//vector<vector<Eigen::Vector3d>> vvCamPosture; // �������ɨ�����̬
//vector<Eigen::Vector6d> vGripperPos; // ��е��ĩ��ɨ���λ��
//vector<vector<Eigen::Vector3d>> vvGripperPosture; // ��е��ĩ��ɨ�����̬
vector<Eigen::Vector3d> vDrawLinePoint; // ���������
vector<Eigen::Vector2i> vDrawLineIndex; // ֱ�ߵ��������ж�Ӧ������
Eigen::Matrix3d rotateOfBox; // ��ת���Ƶľ���
Eigen::Vector3d rotateCenterPoint(0, 0, 0); // ��ת���Ƶ�����
bool needRotate = false;

Mat_<double> ThermalMatrix = (Mat_<double>(3, 4) <<
    //361.5808854218389, 0, 328.319729617155, // 02+ 01
    //0, 360.6635665050927, 240.1939848387075,
    //0, 0, 1
    //369.2985070506613, 0, 330.4892366917894, // 03+ 01
    //0, 368.2180327076736, 235.0215218107861,
    //0, 0, 1
    //359.0074648969985, 0, 328.5013837903426, // 04+ 01
    //0, 358.4140348092706, 235.2678698630961,
    //0, 0, 1
    //360.4528203437661, 0, 327.7198691308573, // 05+ 01
    //0, 360.2436415204507, 232.4753748600872,
    //0, 0, 1
    360.0275158416254, 0, 327.1226012039918, // 06 + 01
    0, 359.61014275028, 235.4357143440756,
    0, 0, 1
    );           
Mat_<double> H_Thermal2Gripper = (Mat_<double>(4, 4) <<
    //0.9944260889119501, 0.1054315194996837, -0.000973851770097344, -3.090483446211227, // 02 + 01
    //-0.1054123318751893, 0.9943560578464163, 0.01201126607686018, -3.871170102911854,
    //0.002234721440639124, -0.01184166036170611, 0.9999273879137229, 189.472303784103,
    //0, 0, 0, 1
    //0.9936977734985963, 0.1120569719517644, -0.002823115465897441, -4.519516810759166, // 03 + 01
    //-0.1120120781165824, 0.9936236369364042, 0.01285933424705302, -3.734243164560581,
    //0.00424609231375681, -0.0124620687798726, 0.9999133300150513, 165.0083976562811,
    //0, 0, 0, 1
    //0.9998095804228461, 0.01773912663960316, -0.008131806733851679, 0.365149856346644, // 04 + 01
    //-0.01758358115215939, 0.9996683280482973, 0.0188162581558404, -5.908308326905811,
    //0.008462893627951312, -0.01866968888830038, 0.999789889000812, 195.5372350358526,
    //0, 0, 0, 1
    //0.9998527575393859, 0.01642287483705435, -0.004975180697504029, 0.3021635735619626, // 05 + 01
    //-0.01622044749586711, 0.9991342571289401, 0.03830970261480655, 1.517008956855605, 
    //0.005600028921370672, -0.03822336214264183, 0.9992535285214621, 193.559103190898,
    //0, 0, 0, 1
    0.9998030255406717, 0.01003678762671443, 0.01712229581146311, 7.658605203362811, // 06 + 01
    -0.00998723826854383, 0.9999456950633555, -0.002976910484073321, -4.857607429741982,
    -0.01715124460458624, 0.00280531966076668, 0.9998489710901916, 183.4847317591889,
    0, 0, 0, 1
    );
Mat_<double> CameraMatrix = (Mat_<double>(3, 4) <<
    //523.1525722504659, 0, 312.5062915840161, // 01 + 01
    //0, 523.3577036865665, 238.2842352121016, 
    //0, 0, 1 
    //528.766925558962, 0, 311.9638114429853, // 02 + 01
    //0, 528.8955430536774, 243.0001985080447,
    //0, 0, 1
    //520.0612603324938, 0, 309.8338305909917, // 03 + 01
    //0, 520.2741992134596, 239.2885546845797,
    //0, 0, 1
    //522.5132096646466, 0, 314.4096095142045, // 04 + 01
    //0, 522.6408903112506, 238.9782871382926,
    //0, 0, 1
    //523.862249381732, 0, 312.9416154204922, // 05 + 01
    //0, 523.4951146951618, 240.0907524004895,
    //0, 0, 1
    //517.4779940710687, 0, 317.3100714168498, // 06 + 01
    //0, 517.3162808995542, 235.99663531724,
    //0, 0, 1
    //523.1698064275082, 0, 315.6000149900301, 0,  // 07 + 01
    //0, 522.5236726388443, 238.731200229786, 0,
    //0, 0, 1, 1
    //518.1307311340169, 0, 310.1465025665003, 0, // ��������01
    //0, 517.1740390426112, 246.9771637183361, 0,
    //0, 0, 1, 1
    512.0885265853337, 0, 310.7629199216577, 0, // ��������02
    0, 514.1919439304321, 238.1648678578148, 0,
    0, 0, 1, 1
    );
Mat_<double> H_Camera2Gripper = (Mat_<double>(4, 4) <<
    //0.004662552205638271, 0.9996579161963214, 0.02573540737890359, -269.3796089832269, // 01 + 01
    //-0.9999855178252958, 0.004591815629342966, 0.002807021357480359, 15.39230327699097,
    //0.00268788887510743, -0.02574812255785944, 0.999664848555825, 43.23105925406757,
    //0, 0, 0, 1
    //0.01438367750141423, 0.9938759545835633, 0.1095613833526512, -270.5112909816904, // 02 + 01
    //-0.9997665508853087, 0.0160620434108526, -0.0144517989316703, 16.17100803965559,
    //-0.0161230751542269, -0.1093279363295547, 0.993874966374288, 44.50258041623692,
    //0, 0, 0, 1
    //0.00958604315350764, 0.9953552061176131, 0.09579207394781178, -267.2184161564147, // 03 + 01
    //-0.9999435358126759, 0.009102496835935558, 0.005483588040281089, 13.71244647295271,
    //0.004586170854080411, -0.0958392310377944, 0.9953862611222747, 46.45701964511662,
    //0, 0, 0, 1
    //-0.01586499097665106, 0.9957103368337697, 0.09115496247430185, -267.5637746058447, // 04 + 01
    //-0.9998656020726254, -0.0154219708967791, -0.005562428031455895, 22.58043900363178,
    //-0.004132777910438874, -0.09123095930680272, 0.9958211848573537, 47.84964350721695,
    //0, 0, 0, 1
    //0.008023804343289309, 0.9972416773628504, 0.07378790886309847, -265.5771204911172, // 05 + 01
    //-0.999965261417238, 0.00783535153451298, 0.002843101314633167, 14.33394466412609,
    //0.002257104918978327, -0.07380815806439638, 0.9972699039279816, 41.21047856042247,
    //0, 0, 0, 1
    //-0.01323314158583933, 0.9975237310947807, 0.06907452400499678, -263.4802244469423, // 06 + 01
    //-0.9998881930482328, -0.01368226867844058, 0.006032986536585495, 11.31791364805999,
    //0.006963143435891113, -0.06898696562799894, 0.9975932604057292, 60.70242570947985,
    //0, 0, 0, 1
    //0.02274594414875231, 0.9973321405551901, 0.06936298321427679, -264.3453399527651, // 07 + 01
    //-0.9997079377920995, 0.02325699122628033, -0.006568978198191029, 5.066745581072112,
    //-0.008164627379705306, -0.06919330729704223, 0.9975698597517106, 43.76288919045668,
    //0, 0, 0, 1
    //-0.001286369320821379, 0.9306192358108915, 0.3659865888153867, -300.5855399553457, // ��������01
    //-0.999976450129698, -0.003664304683179687, 0.005802762893367786, 21.37973428592569,
    //0.006741249140794908, -0.3659705053825258, 0.9306020334977141, 257.6209579677222,
    //0, 0, 0, 1
    -0.01951423799053398, 0.9312611408234442, 0.363829468443481, -305.8995732751089, // ��������02
    -0.9998084430314124, -0.01872487761172303, -0.00569703445038508, 25.12601211384697,
    0.001507235466566337, -0.3638709476595282, 0.9314481529804035, 280.4457280942656,
    0, 0, 0, 1
    );

Mat_<double> CameraMatrix_HD = (Mat_<double>(3, 4) <<
    //3607.34731672557, 0, 2060.678044038478, 0, // ��������01
    //0, 3602.127203698835, 1533.171243767146, 0,
    //0, 0, 1, 1
    3754.085417602176, 0, 2077.680140219795, 0, // ��������02
    0, 3821.86107852316, 1420.24489637215, 0,
    0, 0, 1, 1
    );
Mat_<double> H_Camera2Gripper_HD = (Mat_<double>(4, 4) <<
    //0.01696260080323131, -0.9010870190671114, -0.4333064207264153, 266.6324382848773, // ��������01
    //0.9992372742159346, 0.03052259067895124, -0.0243565448238105, -4.428562294543401,
    //0.03517300088845565, -0.4325627764001339, 0.9009174792851498, 297.1217039628784,
    //0, 0, 0, 1
    //0.01614577371527526, -0.8984860982431586, -0.4387049637910658, 266.2384354263247, // ��������02 bmp
    //0.9992863432166579, 0.02948519165358343, -0.02360990753542463, -3.866555483788407,
    //0.03414847363814341, -0.4380106787932649, 0.8983209487767989, 280.9307856040775,
    //0, 0, 0, 1
    //0.02014267116486401, -0.9009091873018602, -0.43353997397408, 265.831690774124, // ��������02 jpg
    //0.9992024838559621, 0.03309286430735847, -0.0223440951486196, -5.807037271119402,
    //0.03447708013187822, -0.4327441490846947, 0.9008572763643214, 299.3896070291348,
    //0, 0, 0, 1
    //0.02835265973129042, -0.9062595625081382, -0.4217697618947087, 265.0399049799515, // ��������02 bmp cv::CALIB_FIX_K1
    //0.9990040311453963, 0.04023324078680557, -0.01929331726372957, -8.531939648809658,
    //0.0344539176496618, -0.4208026754885816, 0.9064976755956087, 346.5969605585601,
    //0, 0, 0, 1
    //0.01622003255684401, -0.899638432384512, -0.43633427726984, 265.4980165339614, // ��������02 bmp  cv::CALIB_USE_EXTRINSIC_GUESS
    //0.9992742026917313, 0.02962852934957327, -0.0239419732559169, -4.072152491986731,
    //0.03446706222845949, -0.4356292474402059, 0.8994660529425329, 288.0400917437445,
    //0, 0, 0, 1
    //0.01614577371527526, -0.8984860982431586, -0.4387049637910658, 266.2384354263247, // ��������02 bmp cv::CALIB_FIX_K3
    //0.9992863432166579, 0.02948519165358343, -0.02360990753542463, -3.866555483788407,
    //0.03414847363814341, -0.4380106787932649, 0.8983209487767989, 280.9307856040775,
    //0, 0, 0, 1
    0.01314910284754411, -0.896439887183003, -0.4429702357513761, 301.8596209169898, // ��������02 bmp cv::CALIB_FIX_FOCAL_LENGTH
    0.9994328572910263, 0.0255176478343162, -0.02197301563506987, -5.083359733845299,
    0.03100104613396013, -0.442430082969421, 0.8962670120127547, 331.522327587587,
    0, 0, 0, 1
    );

//��е�۲ɼ�����ʱ����̬
vector<string> robotPositionCamera = {
    //"A-2854-5898+4176+179-000-179BC", 001
    //"A+0697-5898+3624+180-029-179BC",
    //"A-2976-5925+3567+178-032+005BC",
    //"A-0003-6652+2606-179-021+091BC",
    //"A-0002-4732+3093+175-022-090BC"

    //// ���μ��������Ķ����е�����ģ���������20cm������ƽ������̱�
    //"A-2578-5791+3916+177+001+179BC",
    //"A+0065-5792+3525+177-019+179BC",
    //"A-5546-5424+1676-179+027-175BC",
    //"A+0058-5880+3059+177-019+093BC"
    //// ���μ��������Ķ����е�����ģ���������2cm���̱�ƽ������̱�
    //"A-0424-4623+3705+178+001+092BC",
    //"A+0410-6049+3499+178-011+091BC",
    //"A-0315-2192+3027-179+017+090BC",
    //"A+0022-6669+3733+179-017+179BC",
	//// ���μ��������Ķ���������37cm
 //   "A-1970-6564+4493+177+001+179BC",
 //   "A+0554-5357+4858-178-012-147BC",
 //   "A+0148-5052+4241-176-008+125BC",
 //   "A+0633-5412+4441+177-015+035BC",
     
     //���˴��ܳ�Լ104.4cm������ࣺ�����˴���β�������ߣ�������е�ۻ��������˴����Ķ�׼�����е㣬���˴�79cm�����ϣ���ͷ����⣩����
    "A+2818-6989+4021-179-030-173BC", // ����� ͷ ---> β 
    "A+2818-6104+4022-179-030-173BC",
    "A+2818-4273+4022-178-030-173BC",
    "A+2337-4307+3817-172-026+172BC", // ���� β ---> ͷ
    "A+2337-6109+3817-172-026+172BC",
    "A+2337-6946+3424-168-006+109BC",
    "A+0517-7176+3390-179-000+090BC", // �м� ͷ ---> β
    "A+0517-5547+3390-179-000+090BC",
    "A+0517-2960+3390-179-000+090BC",
    "A-0230-4389+3784+178-014+002BC",  // ��һ�� β ---> ͷ
    "A-0230-6095+3784+178-014+002BC",
    "A-0438-7955+3762+178-020+002BC",
    "A-2261-7473+3697+177-029-001BC", // ��һ�������  ͷ---> β
    "A-2261-6100+3709+177-029-001BC",
    "A-2261-4250+3709+177-029-001BC",
	//// ���˴��ܳ�Լ104.4cm�����ڲࣺ�����˴���β�������ߣ�������е�ۻ��������˴����Ķ�׼�����е㣬���˴�79cm�����ϣ���ͷ����⣩����
 //   "A-2636-2985+3405+148+004+080BC",  // ����� β ---> ͷ
 //   "A-2636-5089+3405+148+004+080BC",
 //   "A-2636-6519+3405+148+004+080BC",
 //   "A-1322-7197+3658+159+003+090BC", // ���� ͷ ---> β
 //   "A-1322-5620+3658+159+003+090BC",
 //   "A-1322-3377+3658+159+003+090BC",
 //   "A-0597-3280+3658+177-002+091BC", // �м� β ---> ͷ
 //   "A-0597-5248+3641+177-002+091BC",
 //   "A-0581-7659+3660+178+001+091BC",
 //   "A+0696-7382+3657-165+003+093BC",  // ��һ�� ͷ ---> β
 //   "A+0696-5323+3657-165+003+093BC",
 //   "A+0696-3583+3657-165+003+093BC",
 //   "A+2347-3704+2332-149+002+089BC",  // ��һ������� β ---> ͷ
 //   "A+2347-5250+2332-149+002+089BC",
 //   "A+2347-7534+2332-149+002+089BC",
    //// ���˴��ܳ�Լ104.4cm������ࣺ�����˴���β�������ߣ������������½�
    //"A+2558-1636+3937-143-008+179BC",  // ����� ͷ ---> β
    //"A+0492-1636+3937-143-008+179BC",
    //"A-1983-1636+3937-143-008+179BC",
    //"A-1983-3814+3937-159-004-179BC", // ���� β ---> ͷ
    //"A+0496-3814+3937-159-004-179BC",
    //"A+2309-3814+3937-159-004-179BC",
    //"A+3374-6247+4258-179-001-178BC", // �м� ͷ ---> β
    //"A+0440-6247+4258-179-001-178BC",
    //"A-2272-6247+4258-179-001-178BC",
    //"A-2271-7101+4232-179+000-179BC", // ��һ�� β ---> ͷ 
    //"A+0413-7101+4232-179+000-179BC",
    //"A+1924-7101+4232-179+000-179BC", // ��һ������� --- ����
};
vector<string> robotPositionThermal = {
};

//��������ָ��cv::Mat
std::shared_ptr<open3d::geometry::PointCloud> pcl_ptr = std::make_shared<open3d::geometry::PointCloud>();
//std::shared_ptr<open3d::geometry::VoxelGrid> voxel_ptr = std::make_shared<open3d::geometry::VoxelGrid>();
std::shared_ptr<open3d::geometry::VoxelGrid> pcl_voxel = std::make_shared<open3d::geometry::VoxelGrid>();

vector<string> rgbdStringInfo = {
	//// �����������˴�01
 //   "A+040902-088639-002369-17636-00063-17112BC",
 //   "A+014142-087284-002370-17479+00641-17105BC",
 //   "A+009872-096636-000668+17727+00708-17117BC",
 //   "A+013676-076808-000673-16586+00452-17801BC"
    // �����������˴�02
    "A-000863-078105+021798-16135+00491-17944BC",
    "A+008166-094561+019856-17580+00313+17783BC",
    "A-000864-099900+021778-17412+00266-17708BC",
    "A-017726-099900+020778-17312+00364-17708BC",
};
vector<string> hdrgbStringInfo = rgbdStringInfo;
//vector<string> hdrgbStringInfo = {
//    // ��ʯtest
//    "A+2818-6989+4021-179-030-173BC", // ����� ͷ ---> β 
//    "A+2818-6104+4022-179-030-173BC",
//    "A+2818-4273+4022-178-030-173BC",
//};
vector<cameraImgInfo> vRGBDInfo(rgbdStringInfo.size());
vector<bboxImgInfo> vRGBDBboxInfo(rgbdStringInfo.size());
vector<cameraImgInfo> vHDRGBInfo(hdrgbStringInfo.size());
vector<bboxImgInfo> vHDBboxInfo(hdrgbStringInfo.size());
vector<cameraImgInfo> vThermalInfo(robotPositionThermal.size());

int main() {
#pragma comment(lib, "comsuppw.lib") // ���char* ת wchar_t����
#pragma comment(lib, "Ws2_32.lib") // �޷�����inet_addr

    ////agi._oriAgvPos = Eigen::Vector3d{ 1000.,1000.,45. * CV_PI / 180 };
    //agi._oriAgvPos = Eigen::Vector3d{ 0.,0.,45. * CV_PI / 180 };
    //Eigen::Vector3d diff{ 1000., 1000., 90. * CV_PI / 180 };
    ////Eigen::Vector3d diff{ 0., 0., 90. * CV_PI / 180 };
    ////Eigen::Vector3d diff{ 0., 0., -90. * CV_PI / 180 };
    //Eigen::Vector3d pos{ 100., 100., 100. };
    //Eigen::Vector3d posRT = AGVMove2ArmPos(diff, pos, true);
    //cout << "ת��������Ϊ��" << posRT.transpose() << endl << endl << endl;
    //Eigen::Vector3d diff2{ 1000., 1000., -45. * CV_PI / 180 };
    ////Eigen::Vector3d diff2{ 0., 0., 90. * CV_PI / 180 };
    //Eigen::Vector3d pos2{ 100., 100., 100. };
    //posRT = AGVMove2ArmPos(diff2, pos2, false);
    //cout << "ת��������Ϊ��" << posRT.transpose() << endl;
    //return 0;

    //agi._worldGoalPos = Eigen::Vector3d { 3000., 3000., 1000 };
    //CalcAgvPosForRGBD(500, 1500, 60.);
    //return 0;

#pragma region Conect Camera and Robot

#if RGBD_CAMERA

    // ��ʼ��������
    astraCameraD2C = new AstraCameraD2C();
    if (astraCameraD2C->CameraInit(HARDWARE_D2C) != CAMERA_STATUS_SUCCESS) {
        printf("camera init failed\n");
        return -1;
    }

#endif // RGBD_CAMERA

#if HDRGB_CAMERA
#if 1

    if (!init_mindvision()) {
        printf("mindvision camera init fail! \n");
        system("pause");
        return 0;
    }
    //char hdPath[100];
    //sprintf(hdPath, "%s%s%04d", dataPath, "recollect/exposuretime/", i);
    //save_mindvision(hdPath);
#endif
#endif // HD RGB mindvision

#if ROBOT_TYPE == 0 && ROBOT_OFFLINE_TEST == 1
    while (1) {
        char portChar[10] = "COM";
        cout << "�������е��ͨ�Ŵ��ڣ�";
        int portNum;
        cin >> portNum;
        sprintf_s(portChar, "COM%d", portNum);
        bool retsp = sp.Serial_open(portChar, 115200);
        if (!retsp) {
            cout << "���ڳ�ʼ�����󡣡���" << endl;
        }
        else {
            break;
        }
    }
#elif ROBOT_TYPE == 1 && ROBOT_OFFLINE_TEST == 1
    try {
        rokaeRobot = new XMateRobot(rokaeIP);
        //XMateRobot rokaeRobot(rokaeIP); // ����xMate6�����
        auto robotinfo = rokaeRobot->robotInfo(ec);
        std::cout << "�������汾��: " << robotinfo.version << ", ���ͣ�" << robotinfo.type << std::endl;
        std::cout << "RokaeSDK�汾: " << rokaeRobot->sdkVersion() << std::endl;
        //WaitRokaeRobot(rokaeRobot);
    }
    catch (exception e) {
        cout << "δ�����ϻ�����!" << endl;
        return 0;
    }

#endif // ROBOT_ARM

#if AGV_CONTROL

    InitInternet();
    if (ConnectSeer(19210, m_SockClient10)) {
        cout << "19210���ӳɹ�..." << endl;
    }
    else {
        cout << "19210����ʧ��..." << endl;
    }
    if (ConnectSeer(19204, m_SockClient04)) {
        cout << "19204���ӳɹ�..." << endl;
    }
    else {
        cout << "19204����ʧ��..." << endl;
    }
    if (ConnectSeer(19206, m_SockClient06)) {
        cout << "19206���ӳɹ�..." << endl;
    }
    else {
        cout << "19206����ʧ��..." << endl;
    }

#endif // AGV_control

#pragma endregion

    //#pragma region ��ȡ��ʼ����   Ԥ��һЩ���ȶԻ�������Ԥɨ   ��ȡ����
    //
    //    system("cls");
    //    cout << "-------------------------------ͼ��Ԥ�ɼ��ɼ������������ַ�����-------------------------------" << endl;
    //
    //    string str;
    //    cin >> str;
    //
    //    // ͼƬԤ�ɼ�
    //    string rgbPath = "./imgs/rgbd_frame/rgb/0000.jpg";
    //    string depthPath = "./imgs/rgbd_frame/depth/0000.png";
    //
    //    cv::Mat colorSaveTemp(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_8UC3);
    //    cv::Mat depthSaveTemp(IMAGE_HEIGHT_480, IMAGE_WIDTH_640, CV_16UC1);//640x480
    //    cv::Mat colorImageDup;
    //    cv::Mat depthImageDup;
    //    if (astraCameraD2C->GetStreamData(colorSaveTemp, depthSaveTemp) == CAMERA_STATUS_SUCCESS) {
    //        flip(colorSaveTemp, colorImageDup, 1); // ������ͼƬ����
    //        flip(depthSaveTemp, depthImageDup, 1);
    //        //cv::imshow("a", colorImageRealTime);
    //        //cv::waitKey(0);
    //        cv::imwrite(rgbPath, colorImageDup); // ���浱ǰRGB�����
    //        cv::imwrite(depthPath, depthImageDup); // PNG16
    //    }
    //
    //#pragma endregion


    // ��е�۸�λ
    MoveToOnePoint("A+063125-017411+027699+00670+07869+00360BC", &sp);

#if  COLLECT_RGBD

#if INTERACTION

    // ��ȡʵʱRGBD���������ݣ��˻�������ȡ���Ŀ���
    // ��ʱ��
    Timer* ti = new Timer();
    ti->start(TIMER_TIMING_TIME, TimerInterpute); // 20ms
    cameraImgInfo currImgInfo;
    CollectImgAndInteraction(currImgInfo); // ��ʾ���˻�����
    ti->stop();
    ti->~Timer();

#endif


#if VLID_INFERENCE

    // �ڵ���python VILD�ű���������
    bool ret = CallPythonScriptInference(false);

    string vildDepthPath = "./imgs/rgbd_frame/depth/current_frame.png";
    string vildInferenceResult = "./imgs/rgbd_frame/vild_result/current_frame/";
    // ����bbox��Ϣ
    IOFile iofBBox(vildInferenceResult + "box.txt", 0);
    vector<vector<float>> vvTotalBBox;
    iofBBox.ReadFile2FloatArr(vvTotalBBox);

    vector<Eigen::Vector4i> v4DBoxPos(vvTotalBBox.size(), Eigen::Vector4i(0, 0, 0, 0));
    vector<cv::Rect> v4DBoxPosRect(vvTotalBBox.size(), cv::Rect(0, 0, 0, 0));
    vector<cv::Mat> vildBBoxImageVec; // VILD����ģ
    // ����BBox��Ӧ����ģ��Ϣ
    for (int i = 0; i < vvTotalBBox.size(); i++) {
        //for (int j = 0; j < vvTotalBBox[i].size(); j++) {
        //    cout << vvTotalBBox[i][j] << " ";
        //}
        //cout << endl;
        v4DBoxPos[i] = Eigen::Vector4i((int)vvTotalBBox[i][0], (int)vvTotalBBox[i][1], (int)vvTotalBBox[i][2], (int)vvTotalBBox[i][3]);
        v4DBoxPosRect[i] = cv::Rect((int)vvTotalBBox[i][1], (int)vvTotalBBox[i][0], ((int)vvTotalBBox[i][3] - (int)vvTotalBBox[i][1]), ((int)vvTotalBBox[i][2] - (int)vvTotalBBox[i][0]));
        char fileName[12];
        sprintf_s(fileName, "%d.jpg", i);
        string maskImagePath = vildInferenceResult + string(fileName); // ".jpg"
        Mat srcMask = imread(maskImagePath);
        cv::Mat dst(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8U);
        Mat gray;
        cv::cvtColor(srcMask, gray, COLOR_BGR2GRAY);
        cv::threshold(gray, dst, 128, 255, THRESH_BINARY);
        //cv::imshow("bbox", dst);
        //cv::waitKey(0);
        vildBBoxImageVec.push_back(dst);
    }
    vildBBoxImageVec.swap(vildBBoxImageVec);

    //return 0;

    // ѡ������bbox
    // �۸��������������ѡ�����2Dbbox��VILD��
    vector<int> vPixelInBBoxIndex;
    int validPixelInBBox = -1;
    for (int i = 0; i < vildBBoxImageVec.size(); i++) {
        bool ret = PixelInBBox(v4DBoxPosRect[i], cv::Point{ mouseClickCol, mouseClickRow });
        if (ret) {
            //cout << "��" << i << "��bbox�ڵ����Χ��" << endl;
            vPixelInBBoxIndex.emplace_back(i);
        }
        if (vildBBoxImageVec[i].at<uchar>(mouseClickRow, mouseClickCol) == 255) validPixelInBBox = i;
    }
    cout << "��" << vPixelInBBoxIndex.size() << "��BBox��Ч����ӽ�bboxΪ��" << validPixelInBBox << endl;
    if (validPixelInBBox == -1 && vPixelInBBoxIndex.empty()) {
        cout << "����Ϊ��..." << endl;
        return 0;
    }
    else if (validPixelInBBox == -1 && !vPixelInBBoxIndex.empty()) {
        validPixelInBBox = vPixelInBBoxIndex[0];
    }
    cout << "��ӽ�bbox�ĳ�����Ϣ��" << v4DBoxPosRect[validPixelInBBox].width << "   " << v4DBoxPosRect[validPixelInBBox].height << endl;

    // ����ƽ�����
    cv::Mat vildDetphImg = cv::imread(vildDepthPath, cv::IMREAD_ANYDEPTH);
    double agvDepth = CountImageAgvDepth(vildDetphImg, vildBBoxImageVec[validPixelInBBox]);

    //cv::Scalar agvDepth = cv::mean(vildDetphImg, vildBBoxImageVec[validPixelInBBox]);
    //cout << "ƽ�����Ϊ��" << agvDepth[0] << "   " << cv::mean(vildDetphImg) << " " << agvDepth2 << endl;
    //// ��ǰ����ͼ��agvDepth���ĳ�����
    ////double totalImgLength = 2. * tan(HORIZON_ANGLE * CV_PI / 180) * agvDepth[0];
    ////double totalImgWidth = 2. * tan(VERTICAL_ANGLE * CV_PI / 180) * agvDepth[0];
    //double totalImgLength = 2. * tan(HORIZON_ANGLE * CV_PI / 180) * agvDepth2;
    //double totalImgWidth = 2. * tan(VERTICAL_ANGLE * CV_PI / 180) * agvDepth2;
    ////cout << "��ǰ����ͼ��agvDepth���ĳ�����:" << totalImgLength << "  " << totalImgWidth << endl;
    //double objectWidth = totalImgWidth * (double)v4DBoxPosRect[validPixelInBBox].width / IMAGE_WIDTH;
    //double objectLength = totalImgLength * (double)v4DBoxPosRect[validPixelInBBox].height / IMAGE_HEIGHT;
    //cout << "��ǰĿ��ĳ�����:" << objectLength << "  " << objectWidth << endl;
    // ��ά�ع�ȷ��Ŀ����·�Χ
    std::shared_ptr<open3d::geometry::PointCloud> curr_ptr = std::make_shared<open3d::geometry::PointCloud>();
    *curr_ptr = *ReconstructFromOneImg(currImgInfo, vildBBoxImageVec[validPixelInBBox]);
    open3d::visualization::DrawGeometries({ curr_ptr }, "point cloud");
    open3d::geometry::AxisAlignedBoundingBox bbox = curr_ptr->GetAxisAlignedBoundingBox();
    cout << "bbox info: " << bbox.GetExtent().transpose() << endl;
    Eigen::Vector3d objInfoLWH = bbox.GetExtent();
    Eigen::Vector3d objInfoCenter = bbox.GetCenter();
    //return 0;

#endif

    // ����ƽ�����
    double agvDepth = currImgInfo._DepthImg.at<ushort>(mouseClickRow, mouseClickCol);
    // AGV����
    Eigen::Vector3d goal = ConvertPixel2World(mouseClickRow, mouseClickCol, currImgInfo, agvDepth);
    //Eigen::Vector3d goal = ConvertPixel2World(mouseClickRow, mouseClickCol, currImgInfo, currImgInfo._DepthImg.at<ushort>(mouseClickRow, mouseClickCol));
    //cout << "��ǰ��ά�ع��õ���Ŀ�����Ϣ��" << goal[0] << "    " << goal[1] << "    " << goal[2] << endl;
    if (agvDepth < MIN_DISTANCE || agvDepth > MAX_DISTANCE) {
        cout << "��ǰ���ֵ��Ч!" << endl;
        return 0;
    }
    //if (agvDepth == 0) {
    //    cout << "��ǰ���ֵ��Ч!" << endl;
    //    return 0;
    //}
    //return 0;


    AMRLocalInfo currAmrli;
    // ��ȡ��ǰagv���ԭʼλ��
    RequestAMRLocal(currAmrli);
    currAmrli._x *= 1000.;
    currAmrli._y *= 1000.;
    //cout << "-----------------" << currAmrli._x << "   " << currAmrli._y << endl;
    agi._currAgvPos = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    agi._worldGoalPos = goal;
    agi._oriAgvPos = Eigen::Vector3d(currAmrli._x, currAmrli._y, currAmrli._angle); // ԭ���Ӧ��AGVλ��
    //agi._oriArmPos = Eigen::Vector3d(curr6DMatPose.at<double>(0, 0), curr6DMatPose.at<double>(0, 1), curr6DMatPose.at<double>(0, 2));

    cv::Mat tureGoal = arm2agv(currAmrli._x, currAmrli._y, currAmrli._angle) * (cv::Mat_<double>(4, 1) << goal[0], goal[1] - 100., goal[2], 1);
    std::shared_ptr<open3d::geometry::PointCloud> tmpNowPointCloud = std::make_shared<open3d::geometry::PointCloud>();
    reconstructInfo ri;
    *tmpNowPointCloud = *ReconstructFromOneImg(currImgInfo, ri);
    // �滮RGBD����������ʾ
    vector<Eigen::Vector3d> vGoal = CalcAgvPosForRGBD(400, 1200, 30.);
    //cout << vGoal[0].transpose() << "   " << vGoal[1].transpose() << vGoal[2].transpose() << endl;
    //cout << "����2��" << (Eigen::Vector2d(vGoal[0][0], vGoal[0][1]) - Eigen::Vector2d(goal[0], goal[1])).norm() << endl;
    vector<Eigen::Vector3d>vEightPoint;
    vector<Eigen::Vector2i>vLineIndexTemp;
    vEightPoint.emplace_back(Eigen::Vector3d(0., 0., 0.)); // P0
    vEightPoint.emplace_back(vGoal[0]); // P1
    vEightPoint.emplace_back(vGoal[1]); // P2
    vEightPoint.emplace_back(vGoal[2]); // P3
    vEightPoint.emplace_back(goal); // P3
    //vEightPoint.emplace_back(Eigen::Vector3d{ goalAgv.at<double>(0, 0), goalAgv.at<double>(1, 0), goalAgv.at<double>(2, 0) }); // P3
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 1));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 2));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 3));
    vLineIndexTemp.emplace_back(Eigen::Vector2i(0, 4));
    std::shared_ptr<open3d::geometry::LineSet> lineSetTemp_cloud = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vEightPoint, vLineIndexTemp));
    //open3d::visualization::DrawGeometries({ lineSetTemp_cloud, tmpNowPointCloud }, "BoundingBox");

    //RobotArm2Goal(); // ���λ��ָ��Ŀ��
    //return 0;

    RobotArmReset(); // ��е��λ�˸�λ
    //thread reconThread(UpdateRealTimePointCloud); // ���߳�ʵʱ�жϵ�ǰRGBD�Ƿ��·����ͻ
    //reconThread.detach();
    AMRUnlock(0, 1, m_SockClient10);//��բ��
    // ����1��AGV n����λ����е��ָ��Ŀ��
    vector<cameraImgInfo> vCameraInfo(vGoal.size());
    vector<Eigen::Vector3d> vAgvInfo(vGoal.size());
    //for (int goalPos = 0; goalPos < 1; goalPos++) {
    for (int goalPos = 0; goalPos < vGoal.size(); goalPos++) {
        Eigen::Vector3d currAgvGoal = vGoal[goalPos];
        //cout << "��ǰAGVĿ��㣺" << currAgvGoal.transpose() << endl;
        Move2Goal(
            Eigen::Vector2d{ tureGoal.at<double>(0, 0), tureGoal.at<double>(1, 0) },
            Eigen::Vector2d{ currAgvGoal[0], currAgvGoal[1] }
        ); // AGV·���滮�ƶ���Ŀ��

        RobotArm2Goal(); // ���λ��ָ��Ŀ��// �̶�����ROBOT_UPDATE_MINDIS����3D������Ϣ������-�����ػ�-��·���Ƿ����ã������ã����±��BFS����·�������ã������ߣ�

        AMRLocalInfo currAmrli;
        // ��ȡ��ǰagv���ԭʼλ��
        RequestAMRLocal(currAmrli);
        currAmrli._x *= 1000.;
        currAmrli._y *= 1000.;
        vAgvInfo[goalPos] = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };

        cout << "��ǰAGV�㣺" << vAgvInfo[goalPos].transpose() << "��ǰĿ��㣺" << currAgvGoal.transpose() << endl;
        
        cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
        cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
        if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
            long t1 = GetTickCount();
            //cout << "��ȡ������ͼƬ�ɹ���" << endl;
            Mat colorImageDup, depthImageDup;
            flip(colorImage, colorImageDup, 1);
            flip(depthImage, depthImageDup, 1);
            // ��ȡ��ǰ��е������
            string currStrPose;
            sp.ReadRobotArmPosString(sp, currStrPose);
            // ���µ�ǰʵʱͼƬ��Ϣ
            //mutWritePos.lock();
            cameraImgInfo currentDepthImgInfo;
            UpdateRealTimePosInfo(colorImageDup, depthImageDup, currStrPose, currentDepthImgInfo);
            vCameraInfo[goalPos] = currentDepthImgInfo;
            //mutWritePos.unlock();
        }
    }
    
    //// ����2��AGV 1����λ����е��y����̶����ȣ�300mm��ѡ��n����λָ��Ŀ�꣨����n=3��
    //int nPoint = 3;
    //vector<cameraImgInfo> vCameraInfo(nPoint);
    //vector<Eigen::Vector3d> vAgvInfo(nPoint);
    //Eigen::Vector3d currAgvGoal = vGoal[1]; // �����������ĵ�
    //Move2Goal(
    //    Eigen::Vector2d{ tureGoal.at<double>(0, 0), tureGoal.at<double>(1, 0) },
    //    Eigen::Vector2d{ currAgvGoal[0], currAgvGoal[1] }
    //); // AGV·���滮�ƶ���Ŀ��

    //{
    //    // ��ȡ��ǰagv���ԭʼλ��
    //    AMRLocalInfo currAmrli;
    //    RequestAMRLocal(currAmrli);
    //    currAmrli._x *= 1000.;
    //    currAmrli._y *= 1000.;
    //    vAgvInfo[0] = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    //    vAgvInfo[1] = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    //    vAgvInfo[2] = Eigen::Vector3d{ currAmrli._x, currAmrli._y, currAmrli._angle };
    //}

    //RobotArm2Goal();
    //// ��ȡ��ǰ��е������
    //string currStrPose;
    //sp.ReadRobotArmPosString(sp, currStrPose);
    //Eigen::Vector3d position = Eigen::Vector3d(
    //    // "A+063125-017411+027699+00670+07869+00360BC"
    //    atoi(currStrPose.substr(1, 7).c_str()) / 100.,
    //    atoi(currStrPose.substr(8, 7).c_str()) / 100.,
    //    atoi(currStrPose.substr(15, 7).c_str()) / 100.
    //);
    //cout << "�����������꣺" << position << endl;
    //
    //for (int i = 0; i < nPoint; i++) {
    //    string currStrPose;
    //    sp.ReadRobotArmPosString(sp, currStrPose);
    //    cv::Mat colorImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
    //    cv::Mat depthImage(IMAGE_HEIGHT, IMAGE_WIDTH, CV_16UC1);//640x480
    //    if (astraCameraD2C->GetStreamData(colorImage, depthImage) == CAMERA_STATUS_SUCCESS) {
    //        long t1 = GetTickCount();
    //        //cout << "��ȡ������ͼƬ�ɹ���" << endl;
    //        Mat colorImageDup, depthImageDup;
    //        flip(colorImage, colorImageDup, 1);
    //        flip(depthImage, depthImageDup, 1);
    //        // ��ȡ��ǰ��е������
    //        string currStrPose;
    //        sp.ReadRobotArmPosString(sp, currStrPose);
    //        // ���µ�ǰʵʱͼƬ��Ϣ
    //        //mutWritePos.lock();
    //        cameraImgInfo currentDepthImgInfo;
    //        UpdateRealTimePosInfo(colorImageDup, depthImageDup, currStrPose, currentDepthImgInfo);
    //        vCameraInfo[i] = currentDepthImgInfo;
    //    }
    //    if (i != 2) {
    //        string cmd = currStrPose;
    //        int y = 0;
    //        if (i == 1) y = position[1] + 300;
    //        else y = position[1] - 300;
    //        cmd[8] = y > 0 ? '+' : '-';
    //        cmd[9] = abs(y) / 1000 + '0';
    //        cmd[10] = ((abs(y) / 100) % 10) + '0';
    //        cmd[11] = ((abs(y) / 10) % 10) + '0';
    //        cmd[12] = ((abs(y) / 1) % 10) + '0';
    //        cout << "�������㣺" << cmd << "   " << y << endl;
    //        MoveToOnePoint(cmd, &sp);
    //        RobotArm2Goal();
    //    }
    //}

    std::shared_ptr<open3d::geometry::PointCloud> tmpNowPointCloud_2 = std::make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < vCameraInfo.size(); i++) {
        std::shared_ptr<open3d::geometry::PointCloud> tmp = std::make_shared<open3d::geometry::PointCloud>();
        reconstructInfo ri;
        *tmp = *ReconstructFromOneImg(vCameraInfo[i], ri);
        if (i != 0) {
            for (int pd = 0; pd < tmp->points_.size(); pd++) {
                Eigen::Vector3d diff = vAgvInfo[i] - vAgvInfo[0];
                Eigen::Vector3d convertPoint = AGVMove2ArmPos(diff, tmp->points_[pd], true, vAgvInfo[0][2]);
                tmp->points_[pd] = convertPoint;
            }
        }
        *tmpNowPointCloud_2 += *tmp;
    }

    // ��������
    // ·��
    string savePath = "./imgs/data/";
    fstream fsdata(savePath + "info.txt", std::ios::out | std::ios::trunc);
    // agv��ʼλ�á�Ŀ�����Գ�ʼ
    fsdata << 0 << endl;
    fsdata << agi._oriAgvPos[0] << "," << agi._oriAgvPos[1] << "," << agi._oriAgvPos[2] << endl;
    for (int i = 0; i < vCameraInfo.size(); i++) {
        fsdata << i + 1 << endl;
        fsdata << vCameraInfo[i]._poseStr << endl;
        cv::imwrite(savePath + to_string(i + 1) + ".jpg", vCameraInfo[i]._RgbImg);
        cv::imwrite(savePath + to_string(i + 1) + ".png", vCameraInfo[i]._DepthImg);
        fsdata << vAgvInfo[i][0] << "," << vAgvInfo[i][1] << "," << vAgvInfo[i][2] << endl;
    }
    fsdata.close();

    open3d::io::WritePointCloudToPCD(savePath + "pointcloud.pcd", *tmpNowPointCloud_2);
    open3d::io::WritePointCloudToPLY(savePath + "pointcloud.ply", *tmpNowPointCloud_2);
    open3d::visualization::DrawGeometries({ tmpNowPointCloud_2 }, "pcl");

    //RobotArmReset(); // ��е��λ�˸�λ
    ////thread thRobotMove(RobotArm2Goal); // ���߳�ʵʱ������е��ĩ��ָ��Ŀ��
    ////thRobotMove.detach();
    //thread reconThread(UpdateRealTimePointCloud); // ���߳�ʵʱ�жϵ�ǰRGBD�Ƿ��·����ͻ
    //reconThread.detach();
    //AMRUnlock(0, 1, m_SockClient10);//��բ��
    //Move2Goal(goal); // AGV·���滮�ƶ���Ŀ��

    while (agi._move2goal != 3);
    //RobotArm2Goal(); // ���λ��ָ��Ŀ��
    string str;
    cin >> str;
    
#else

    //return 0;

    vector<cameraImgInfo> vCameraInfo;
    vector<Eigen::Vector3d> vAgvInfo;

    // ��ȡtxt����
    string dataPath = "./imgs/data/";
    fstream fsread(dataPath + "info.txt", ios::in);
    string currLine;
    vector<double> currAgvInfo; // ��ǰAGVλ��
    string currArmInfo; // ��ǰ��е��λ��
    int infoIdx = 0;
    while (getline(fsread, currLine)) {
        //cout << currLine << endl;
        if (currLine.length() <= 2) {
            // ��ȡ���
            infoIdx = atoi(currLine.c_str());
            continue;
        }
        if (currLine[0] == 'A' && currLine[40] == 'B' && currLine[41] == 'C') {
            // ��ȡͼƬ��Ӧ��е��λ��
            currArmInfo = currLine;
            continue;
        }
        // ��ȡͼƬ��ӦAGVλ��
        currAgvInfo.clear();
        while (1) {
            int idx = currLine.find(',');
            if (idx == -1) break;
            currAgvInfo.emplace_back(atof(currLine.substr(0, idx).c_str()));
            currLine = currLine.substr(idx + 1);
        }
        currAgvInfo.emplace_back(atof(currLine.c_str()));
        //cout << currAgvInfo[0] << "    " << currAgvInfo[1] << "    " << currAgvInfo[2] << endl;
        if (infoIdx == 0) {
            // agv��ʼλ��
            agi._oriAgvPos = Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] };
        }
        else {
            // ͼƬ��Ӧλ��
            vAgvInfo.emplace_back(Eigen::Vector3d{ currAgvInfo[0], currAgvInfo[1], currAgvInfo[2] });
            cv::Mat rgb = cv::imread(dataPath + to_string(infoIdx) + ".jpg");
            cv::Mat depth = cv::imread(dataPath + to_string(infoIdx) + ".png", cv::IMREAD_ANYDEPTH);
            cameraImgInfo cii;
            IntegrateImgInfo(rgb, depth, currArmInfo, 0, cii);
            vCameraInfo.emplace_back(cii);
        }
    }
    fsread.close();

#pragma region ���ݶ�ȡ

    vector<cameraImgInfo> vRGBDInfoSL(vCameraInfo.size());
    vector<bboxImgInfo> vRGBDVildInfoSL(vCameraInfo.size());
    // ��ȡRGBD��Ϣ
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        //char fileName[10];
        //sprintf_s(fileName, "%04d", i);
        // depth����
        string depthImagePath = dataPath + to_string(i + 1) + ".png"; // ".png"
        //string depthImagePath = dataPath + string(fileName) + ".png"; // ".png"
        cv::Mat srcDepth = imread(depthImagePath, IMREAD_ANYDEPTH);
        // rgb����
        string colorImagePath = dataPath + to_string(i + 1) + ".jpg"; // ".jpg"
        //string colorImagePath = dataPath + string(fileName) + ".jpg"; // ".jpg"
        cv::Mat srcColor = imread(colorImagePath);
        // ����
        cameraImgInfo currCamera;
        IntegrateImgInfo(srcColor, srcDepth, vCameraInfo[i]._poseStr, 0, currCamera);
        vRGBDInfoSL[i] = currCamera;
    }

    // ��ȡRGBD CLIP������BBox
    for (int i = 0; i < vRGBDVildInfoSL.size(); i++) {
        char fileName[10];
        sprintf_s(fileName, "%04d", i);
        // Ŀ���������򣨵����������򣩣�VILD������
        string vildBboxPath = dataPath + "vild/" + to_string(i + 1) + "/";
        bboxImgInfo bboxInfo;
        IntegrateVildBBoxInfo(vildBboxPath, bboxInfo, 0);
        vRGBDVildInfoSL[i] = bboxInfo;
    }

#pragma endregion


    //////////////////////////////

    // ��ѧϰ������ʼѡȡ������ȷ��bbox����ά����������λ�ã���ͶӰ����ǰ�ӽ������Ŀ��� 
    vRGBDVildInfoSL[0]._selectmask = vRGBDVildInfoSL[0]._img[1]; // pose_3
    vRGBDVildInfoSL[1]._selectmask = vRGBDVildInfoSL[1]._img[3];
    vRGBDVildInfoSL[2]._selectmask = vRGBDVildInfoSL[2]._img[3];

    string pcSavePath = CreateDirUseDate("./imgs/itreation/");
    // �ع�
    vector<shared_ptr<open3d::geometry::PointCloud>> vLastPointCloud(vRGBDInfoSL.size(), nullptr);
    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSL = std::make_shared<open3d::geometry::PointCloud>();
    for (int i = 0; i < vRGBDInfoSL.size(); i++) {
        reconstructInfo ri;
        ri._2whichbase = 0x02 | 0x08; // ת����AGV��������ϵ
        ri._mask = vRGBDVildInfoSL[i]._selectmask;
        ri._agvCurrInfo = vAgvInfo[i];
        //ri._2whichbase = 0x02; // ת����AGV��������ϵ
        //ri._agvCurrInfo = vAgvInfo[i];
        //ri._2whichbase = 0x08; // ��ͼ
        //ri._mask = vRGBDVildInfoSL[i]._selectmask;
        vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // ���ؾֲ�����
        *pclPtrSL += *vLastPointCloud[i]; // ����ƴ��
    }

    // ����
    cout << "��ǰ������ɵ��ƴ�ŵ�·��Ϊ��" << pcSavePath << endl;
    int iterateNum = 7;
    for (int itr = 0; itr < iterateNum; itr++) {
        cout << "***�ڣ�" << itr << "�ε�����ʼ����" << iterateNum << "�ε�������" << endl;
        for (int i = 0; i < vRGBDInfoSL.size(); i++) {
            // �����ļ��б���ͼƬ
            char dirPath[50];
            int _ret = _mkdir((pcSavePath + "/iteration").c_str());
            sprintf_s(dirPath, "/iteration/%d/", i);
            _ret = _mkdir((pcSavePath + string(dirPath)).c_str());
            // ��ȡ��ǰ���λ�á�����
            cv::Mat nowPose = vRGBDInfoSL[i]._CameraPose.clone(); // ��i��ͼƬ�����λ�ˣ�ֱ�Ӹ�ֵΪǳ����
            nowPose.at<double>(1, 3) -= 100.;
            nowPose = arm2agv(vAgvInfo[i]) * nowPose;
            // �ӵ�ǰ����ӽǽ�����ͶӰ��2Dͼ����
            vector<cv::Mat> imageVec = PerspectivePointCloudFromSpecialPose(pclPtrSL, nowPose);
            // ����ԭ�ȵ���ģmaskImageVec
            // �������ͼȥ���쳣��
            //DealMaskRegion(imageVec);
            vRGBDVildInfoSL[i]._selectmask = imageVec[3].clone(); // �����maskImageVec[vIndex[i]] = imageVec[3]��ǳ����
            vector<vector<Point>> maskContours; // α���ͼ��Ӧ��Ĥ
            //cv::imshow("mask", vRGBDVildInfoSL[i]._selectmask);
            //cv::waitKey(0);
            // Ѱ��ͼƬ��Ĥ����
            if (itr == iterateNum - 1) maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 1);
            else if (itr == 0) maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            //else maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 1);
            else maskContours = FindContours(vRGBDVildInfoSL[i]._selectmask, 0);
            vector<vector<Point>> baseContours = FindContours(vRGBDInfoSL[i]._RgbImg, 0); // ԭʼͼƬ����������
            vector<Point> optimumContourSrc, optimumContour;
            // ����ĤͼƬ������ԭʼͼƬ�����������һ��
            double err = FindOptimumContours(baseContours, maskContours, optimumContourSrc, optimumContour);
            vector<vector<Point>> vOptimumContours;
            //if (itr == 0) vOptimumContours.emplace_back(optimumContourSrc);
            //else vOptimumContours.emplace_back(optimumContour);
            vOptimumContours.emplace_back(optimumContour);
            //vOptimumContours.emplace_back(optimumContourSrc);
            cv::Mat maskSave(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8UC3, Scalar(0, 0, 0));
            if (vOptimumContours[0].size() != 0) {
                cv::drawContours(maskSave, vOptimumContours, 0, Scalar(255, 255, 255), -1, 8);
            }
            cv::Mat maskUse(vRGBDInfoSL[i]._RgbImg.rows, vRGBDInfoSL[i]._RgbImg.cols, CV_8U, Scalar(0));
            cv::cvtColor(maskSave, maskSave, cv::COLOR_BGR2GRAY);
            cv::threshold(maskSave, maskUse, 128, 255, cv::THRESH_BINARY);

            vRGBDVildInfoSL[i]._selectmask = maskUse; // 3��ȫ�������۱߽�
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "optimum.jpg", maskSave);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + "revise.jpg", vRGBDVildInfoSL[i]._selectmask);
            cv::imwrite(pcSavePath + "/iteration/" + to_string(i) + "/" + to_string(itr) + ".jpg", imageVec[3]);
            // �ع����µ��� --- ɾ����һ�ν������������һ�ν��
            if (vLastPointCloud[i] != nullptr) {
                // ����һ�εĵ��ƽ���Ƴ�
                pclPtrSL = RemoveSmallerPointCloud(pclPtrSL, vLastPointCloud[i]); // �������ƵĲ��һ���������ȥ��С�ĵ���
            }
            reconstructInfo ri;
            ri._2whichbase = 0x02 | 0x08; // ת����AGV��������ϵ��������Ĥ��ͼ
            ri._mask = vRGBDVildInfoSL[i]._selectmask;
            ri._agvCurrInfo = vAgvInfo[i];
            //ri._2whichbase = 0x08; // ת����AGV��������ϵ��������Ĥ��ͼ
            //ri._mask = vRGBDVildInfoSL[i]._selectmask;
            vLastPointCloud[i] = ReconstructFromOneImg(vRGBDInfoSL[i], ri); // ���ؾֲ�����
            *pclPtrSL += *vLastPointCloud[i]; // ����ƴ��
        }
    }
    // ��������
    fstream fs(pcSavePath + "\\readme.txt", ios::out | ios::app);
    fs << endl << "*�������!" << endl;
    fs.close();
    open3d::io::WritePointCloudToPCD(pcSavePath + "\\shipIterated.pcd", *pclPtrSL);
    open3d::io::WritePointCloudToPLY(pcSavePath + "\\shipIterated.ply", *pclPtrSL);
    open3d::visualization::DrawGeometries({ pclPtrSL }, "IteratedPointCloud");

    //return 0;

    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSrcSL = std::make_shared<open3d::geometry::PointCloud>(*pclPtrSL);
    // ������ת��������AABB�İ�Χ�в���������bbox���ط���OBB�İ�Χ��
    open3d::geometry::AxisAlignedBoundingBox bboxBeforeRotateAxis = pclPtrSL->GetAxisAlignedBoundingBox();
    bboxBeforeRotateAxis.color_ = Eigen::Vector3d{ 1.,0.,0. };
    std::shared_ptr<geometry::AxisAlignedBoundingBox> pclBeforeRotatedPtr = std::make_shared<geometry::AxisAlignedBoundingBox>(bboxBeforeRotateAxis); // ��Χ���Ӧ����
    open3d::geometry::OrientedBoundingBox bboxBeforeRotateOri = pclPtrSL->GetOrientedBoundingBox();
    bboxBeforeRotateOri.color_ = Eigen::Vector3d{ 0.,1.,0. };
    std::shared_ptr<geometry::OrientedBoundingBox> pclBeforeRotatedPtr2 = std::make_shared<geometry::OrientedBoundingBox>(bboxBeforeRotateOri); // ��Χ���Ӧ����

    // ����xyz������
    std::shared_ptr<open3d::geometry::LineSet> originandaxis_cloud = DrawXYZAxisAtOrient(drawAxisInfo{ 600.,800.,1000.,bboxBeforeRotateAxis.GetBoxPoints()[0] });

    //// ������ת�Ƕȣ�OBB��AABB
    //cout << "obb info:" << endl;
    //cout << "rotation:" << bboxBeforeRotateOri.R_ << endl;
    //cout << "extent:" << bboxBeforeRotateOri.extent_.transpose() << endl;
    //cout << "extent:" << bboxBeforeRotateOri.GetRotationMatrixFromAxisAngle(Eigen::Vector3d{ 0.,0.,45. * CV_PI / 180 }) << endl;
    //cout << "extent:" << bboxBeforeRotateOri.GetRotationMatrixFromXYZ(Eigen::Vector3d{ 0.,0.,45. * CV_PI / 180 }) << endl;
    //cout << "obb point:" << endl;
    vector<Eigen::Vector3d> vTotalPoint = bboxBeforeRotateOri.GetBoxPoints();
    //for (auto point : vTotalPoint) {
    //    cout << point.transpose() << endl;
    //}
    Eigen::Vector3d oriX = (vTotalPoint[1] - vTotalPoint[0]).normalized();
    Eigen::Vector3d oriY = (vTotalPoint[2] - vTotalPoint[0]).normalized();
    Eigen::Vector3d oriZ = (vTotalPoint[0] - vTotalPoint[3]).normalized();
    Eigen::Vector3d axisX = Eigen::Vector3d{ 1.,0.,0. };
    Eigen::Vector3d axisY = Eigen::Vector3d{ 0.,1.,0. };
    //cout << "x��x��" << acos(oriX.dot(axisX)) * 180 / CV_PI << endl;
    //cout << "x��y��" << acos(oriX.dot(axisY)) * 180 / CV_PI << endl;
    //cout << "y��x��" << acos(oriY.dot(axisX)) * 180 / CV_PI << endl;
    //cout << "y��y��" << acos(oriY.dot(axisY)) * 180 / CV_PI << endl;
    PointCloudRotateInfo pcri;
    pcri._rotateCenter = bboxBeforeRotateOri.center_;
    if (abs(acos(oriX.dot(axisX)) - acos(oriY.dot(axisY))) < pcri._minRotatAngle * CV_PI / 180.) {
        // �н�С��2�ȣ���Ϊ���ƶԷֿ�Ӱ�첻��
        pcri._isNeedRotate = true;
        pcri._rotateXYZ[2] = -(acos(oriX.dot(axisX)) + acos(oriY.dot(axisY))) / 2.;
        pcri._rotateXYZ[2] = pcri._rotateXYZ[2] > CV_PI / 4. ? pcri._rotateXYZ[2] - CV_PI / 2. : pcri._rotateXYZ[2];
    }
    else if (abs(acos(oriX.dot(axisY)) - acos(oriY.dot(axisX))) < pcri._minRotatAngle * CV_PI / 180.) {
        pcri._isNeedRotate = true;
        pcri._rotateXYZ[2] = -(acos(oriX.dot(axisY)) + acos(oriY.dot(axisX))) / 2.;
        pcri._rotateXYZ[2] = pcri._rotateXYZ[2] > CV_PI / 4. ? pcri._rotateXYZ[2] - CV_PI / 2. : pcri._rotateXYZ[2];
    }
    else {
        cout << "��ǰ������xoyƽ��ƫ�ƹ���" << endl;
        return 0;
    }

    cout << "��z����ת�Ƕ�Ϊ��" << pcri._rotateXYZ.transpose() << endl;
    if (pcri._rotateXYZ[2] < pcri._minRotatAngle * CV_PI / 180.) {
        pcri._isNeedRotate = false;
        cout << "AABB��OBB��ת�Ƕ�С��2�ȣ���������ת��" << endl;
    }
    // obb��x��y��z��
    vector<Eigen::Vector3d> vPoint;
    vector<Eigen::Vector2i> vIndex;
    Eigen::Vector3d obbCenter = vTotalPoint[3];
    vPoint.emplace_back(obbCenter);
    vPoint.emplace_back(obbCenter + oriX * 2000.); // 35��y��
    vPoint.emplace_back(obbCenter + oriY * 1500.); // 36��x��
    vPoint.emplace_back(obbCenter + oriZ * 1000.); // 30��z��
    vIndex.emplace_back(Eigen::Vector2i{ 0,1 });
    vIndex.emplace_back(Eigen::Vector2i{ 0,2 });
    vIndex.emplace_back(Eigen::Vector2i{ 0,3 });
    std::shared_ptr<open3d::geometry::LineSet> lineSetAxis = std::make_shared<open3d::geometry::LineSet>(open3d::geometry::LineSet(vPoint, vIndex));

    std::shared_ptr<open3d::geometry::PointCloud> pclPtrSLRotate = std::make_shared<open3d::geometry::PointCloud>();
    if (pcri._isNeedRotate) {
        auto pclPtrSLRotate = RotatePointCloud(pclPtrSL, pcri);
    }
    else {
        pclPtrSLRotate = std::make_shared<open3d::geometry::PointCloud>(*pclPtrSrcSL);
    }
    open3d::visualization::DrawGeometries({ originandaxis_cloud, lineSetAxis, pclPtrSrcSL, pclBeforeRotatedPtr, pclBeforeRotatedPtr2 }, "BeforeRotatePointCloud");
    open3d::visualization::DrawGeometries({ originandaxis_cloud, lineSetAxis, pclPtrSLRotate, pclBeforeRotatedPtr, pclBeforeRotatedPtr2 }, "AfterRotatePointCloud");

    //return 0;


    //// ��ȡ����
    //open3d::geometry::PointCloud src_pcl;
    //open3d::io::ReadPointCloudFromPLY("./imgs/itreation/23_05_25/25/shipIterated.ply", src_pcl);
    //pcl_ptr = std::make_shared<geometry::PointCloud>(src_pcl);
    //open3d::visualization::DrawGeometries({ pcl_ptr }, "src");

    //// �ֿ�ɨ��
    //// ������Ʒֿ�
    vector<ThermalScanInfo> vScanInfo;
    PointCloudInfo currPcli;
    InitPointCloudInfo(pclPtrSLRotate, currPcli);
    vector<std::shared_ptr<open3d::geometry::PointCloud>> vCropedCloudForPCA = PointCloudPartition(pclPtrSLRotate); // ������вü����ĵ���
    vector<std::shared_ptr<open3d::geometry::LineSet>> vNormOrient = PointCloudPartitionPCA(currPcli, vCropedCloudForPCA, vScanInfo);

    //// ���ƺϲ�����ʾ��ֱ��+=Ҳ����
    //int pointNum = 0;
    std::vector<std::shared_ptr<const geometry::Geometry>> vTotalCloud;
    ////shared_ptr<open3d::geometry::PointCloud> downsampledTotal = totalPointCloud->VoxelDownSample(5);//�Ե��ƽ����²���
    shared_ptr<open3d::geometry::PointCloud> downsampledTotal = pclPtrSLRotate->VoxelDownSample(5);//�Ե��ƽ����²���
    for (int i = 0; i < vNormOrient.size(); i++) {
        vTotalCloud.emplace_back(vNormOrient[i]); // ��������һ����ʾ
        // ��ȡ�豸3D����
        std::shared_ptr<open3d::geometry::LineSet> device_pointcloud = DrawDeviceLinePointCloud(vScanInfo[i]._cameraPosition, vScanInfo[i]._cameraRx, vScanInfo[i]._cameraRy, vScanInfo[i]._cameraRz);
        vTotalCloud.emplace_back(device_pointcloud);
        //// ��ȡ��е�۳��� 
        //Eigen::Vector3d gripperPosTemp(vGripperPos[i][0], vGripperPos[i][1], vGripperPos[i][2]);
        //std::shared_ptr<open3d::geometry::LineSet> gripper_pointcloud = DrawXYZOrient(gripperPosTemp, vvGripperPosture[i][0], vvGripperPosture[i][1], vvGripperPosture[i][2]);
        //vTotalCloud.emplace_back(gripper_pointcloud);
    }
    //vTotalCloud.emplace_back(pclPtrSrcSL);
    vTotalCloud.emplace_back(pclPtrSLRotate);
    vTotalCloud.emplace_back(pclBeforeRotatedPtr);
    vTotalCloud.emplace_back(pclBeforeRotatedPtr2);
    vTotalCloud.emplace_back(originandaxis_cloud);
    open3d::visualization::DrawGeometries({ vTotalCloud }, "CloudWithDir");
    //open3d::io::WritePointCloudToPCD(pcSavePath + "\\shipActivate.pcd", *pcl_ptr);
    //open3d::io::WritePointCloudToPLY(pcSavePath + "\\shipActivate.ply", *pcl_ptr);

    // ���������ת���������������λ����ת��ȥ
    if (pcri._isNeedRotate) {
        // ������Ҫ��ת����������λ����ת�س�ʼλ��
        cv::Mat rotatePclMat = RotatePclMat(pcri._rotateXYZ[2]);
        cout << "��ת����2Ϊ��" << rotatePclMat.inv() << endl;
        // ���㵱ǰ�����������ת����������
        for (int si = 0; si < vScanInfo.size(); si++) {
            // z����ת�������̬Ҳ�ı䣨��Ȼ���������̬������ת������ģ���z����ת���Ƹı���̬Ҳ�ı䣩
            cv::Mat cmeraPosetrue = vScanInfo[si]._cameraPosetrue;
            vScanInfo[si]._cameraPosetrue = rotatePclMat({ 0, 0, 3, 3 }).inv() * cmeraPosetrue;
            // z����ת�����λ��Ҳ�ı�
            Eigen::Vector3d cmeraPosition = vScanInfo[si]._cameraPosition - pcri._rotateCenter;
            cv::Mat cameraRotatedMat = rotatePclMat.inv() * (cv::Mat_<double>(4, 1) << cmeraPosition[0], cmeraPosition[1], cmeraPosition[2], 1);
            vScanInfo[si]._cameraPosition = pcri._rotateCenter + Eigen::Vector3d{ cameraRotatedMat.at<double>(0, 0),cameraRotatedMat.at<double>(1, 0),cameraRotatedMat.at<double>(2, 0) };
            cout << "before rotate:" << vScanInfo[si]._cmdStr << endl;
            IntergateScanInfo(vScanInfo[si]);
            cout << "after rotate:" << vScanInfo[si]._cmdStr << endl;
            //cout << "after rotate:" << vScanInfo[si]._cameraPosition.transpose() << endl;
        }

        // ������ת��ĵ���
        std::vector<std::shared_ptr<const geometry::Geometry>> vTotalCloudRotated;
        for (int i = 0; i < vNormOrient.size(); i++) {
            // ��ȡ�豸3D����
            std::shared_ptr<open3d::geometry::LineSet> device_pointcloud = DrawDeviceLinePointCloud(vScanInfo[i]._cameraPosition, vScanInfo[i]._cameraRx, vScanInfo[i]._cameraRy, vScanInfo[i]._cameraRz);
            vTotalCloudRotated.emplace_back(device_pointcloud);
        }
        vTotalCloudRotated.emplace_back(pclBeforeRotatedPtr);
        vTotalCloudRotated.emplace_back(pclBeforeRotatedPtr2);
        vTotalCloudRotated.emplace_back(pclPtrSrcSL);
        vTotalCloudRotated.emplace_back(originandaxis_cloud);
        open3d::visualization::DrawGeometries({ vTotalCloudRotated }, "ori");
    }

#endif

    return 0;






}
