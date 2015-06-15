//
//  MainVC.m
//  attitude
//
//  Created by Andreas Hauenstein on 2015-01-13.
//  Copyright (c) 2015 AHN. All rights reserved.
//

#import "MainVC.h"
#import <CoreMotion/CMMotionManager.h>
#import <math.h>

#define FREQ 50

#define DEG(x) ((x)*(180.0/M_PI))
#define RAD(x) ((x)*(M_PI/180.0))
#define UNUSED(x) (void)(x)
#define SIGN(x) ((x)>=0?1:-1)
//#define abs(x) ((x)>=0?(x):(-(x)))
#define SQR(x) ((x)*(x))
#define BOUND(x,a,b) { if ((x) > (b)) (x) = (b); if ((x) < (a)) (x) = (a); }
#define ROUND(x) ((x)>=0?(int)((x)+0.5):(int)((x)-0.5))

// Loop abbreviations
#define ILOOP(N) for(i=0;i<(N);i++)
#define JLOOP(N) for(j=0;j<(N);j++)
#define RLOOP(N) for(r=0;r<(N);r++)
#define CLOOP(N) for(c=0;c<(N);c++)


// A 3D rotation matrix
typedef struct lp_matrix {
    float m[3][3];
} LP_matrix;


// Streams for socket communication with server
CFReadStreamRef mReadStream = nil;
CFWriteStreamRef mWriteStream = nil;
NSInputStream *mIStream = nil;
NSOutputStream *mOStream = nil;

@interface MainVC ()

// Labels and Textfields
//--------------------------

// Quaternion from MotionManager
@property (weak, nonatomic) IBOutlet UILabel *lbAngle;
@property (weak, nonatomic) IBOutlet UILabel *lbx;
@property (weak, nonatomic) IBOutlet UILabel *lby;
@property (weak, nonatomic) IBOutlet UILabel *lbz;

// Quaternion from Acceleration
@property (weak, nonatomic) IBOutlet UILabel *lbAngleA;
@property (weak, nonatomic) IBOutlet UILabel *lbXA;
@property (weak, nonatomic) IBOutlet UILabel *lbYA;
@property (weak, nonatomic) IBOutlet UILabel *lbZA;

// User Acceleration (motion)
@property (weak, nonatomic) IBOutlet UILabel *lbUsrAccX;
@property (weak, nonatomic) IBOutlet UILabel *lbUsrAccY;
@property (weak, nonatomic) IBOutlet UILabel *lbUsrAccZ;

// Gravity (static)
@property (weak, nonatomic) IBOutlet UILabel *lbGravX;
@property (weak, nonatomic) IBOutlet UILabel *lbGravY;
@property (weak, nonatomic) IBOutlet UILabel *lbGravZ;

// Sum grav + user
@property (weak, nonatomic) IBOutlet UILabel *lbTotX;
@property (weak, nonatomic) IBOutlet UILabel *lbTotY;
@property (weak, nonatomic) IBOutlet UILabel *lbTotZ;

// Server Connection info
@property (weak, nonatomic) IBOutlet UITextField *txtIPA;
@property (weak, nonatomic) IBOutlet UITextField *txtIPB;
@property (weak, nonatomic) IBOutlet UITextField *txtIPC;
@property (weak, nonatomic) IBOutlet UITextField *txtIPD;
@property (weak, nonatomic) IBOutlet UITextField *txtPort;

@property (weak, nonatomic) IBOutlet UIButton *btnLED;

@property (weak, nonatomic) IBOutlet UIButton *btnLEDFusion;
@property (weak, nonatomic) IBOutlet UIButton *btnLEDGravity;
@property (weak, nonatomic) IBOutlet UIButton *btnLEDAccelerometer;
@property (weak, nonatomic) IBOutlet UIButton *btnLEDMadgwick;

// Motion stuff
//--------------

@property NSOperationQueue *deviceQueue;
@property CMMotionManager *motionManager;

// Other
//-------

@property CMAcceleration userAcc;
@property CMAcceleration gravity;
@property float vdisp; // vertical displacement
@property float vspeed; // vertical speed

@property NSTimer *connectTimeout;
@property BOOL isConnected;
@property enum {USE_FUSION, USE_MADGWICK, USE_GRAVITY, USE_ACCELEROMETER} mode;

@end

@implementation MainVC

#pragma  mark View LifeCycle

//---------------------
- (void)viewDidLoad
//---------------------
{
    __block long i = 0;
    [super viewDidLoad];
    
    _mode = USE_FUSION;
    
    _lbx.text = @"0";
    _lby.text = @"0";
    _lbz.text = @"0";
    _lbAngle.text = @"0";
    self.view.frame = [[UIScreen mainScreen] bounds];
    
    [_txtIPA setDelegate:self];
    [_txtIPB setDelegate:self];
    [_txtIPC setDelegate:self];
    [_txtIPD setDelegate:self];
    [_txtPort setDelegate:self];
    [self initIP];
        
    // Motion manager
    _deviceQueue = [[NSOperationQueue alloc] init];
    _motionManager = [CMMotionManager new];
    if (_motionManager.isDeviceMotionAvailable) {
        //const int FREQ = 25;
        _motionManager.deviceMotionUpdateInterval = 1.0 / FREQ;
        //[self.motionManager startMagnetometerUpdates];
        [self.motionManager
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryZVertical
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical
         startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXTrueNorthZVertical
         toQueue:self.deviceQueue
         withHandler:^(CMDeviceMotion *m, NSError *error)
         {
             [[NSOperationQueue mainQueue] addOperationWithBlock:^{
                 i++;
                 CMRotationRate rotationRate = m.rotationRate;
                 CMCalibratedMagneticField magneticField = m.magneticField;
                 
                 
                 _userAcc.x = -m.userAcceleration.x;
                 _userAcc.y = -m.userAcceleration.z;
                 _userAcc.z = -m.userAcceleration.y;

                 _gravity.x = -m.gravity.x;
                 _gravity.y = -m.gravity.z;
                 _gravity.z = -m.gravity.y;

                 CMAcceleration totAcc;
                 totAcc.x = _userAcc.x + _gravity.x;
                 totAcc.y = _userAcc.y + _gravity.y;
                 totAcc.z = _userAcc.z + _gravity.z;
                 
                 // Call the Madgwick filter
                 // xzy yxz zyx
//                 MahonyAHRSupdate (rotationRate.x, rotationRate.y, rotationRate.z
//                                     ,totAcc.x, totAcc.y, totAcc.z
//                                     ,magneticField.field.x, magneticField.field.y, magneticField.field.z);
                 MadgwickAHRSupdate (rotationRate.x, rotationRate.y, rotationRate.z
                                     ,totAcc.x, totAcc.y, totAcc.z
                                     ,magneticField.field.x, magneticField.field.y, magneticField.field.z);
//                 MadgwickAHRSupdateIMU (rotationRate.x, rotationRate.y, rotationRate.z
//                                        ,totAcc.x, totAcc.y, totAcc.z);

                 CMQuaternion q;
                 if (_mode == USE_MADGWICK) {
//                     q.w = SEq_1;
//                     q.x = SEq_2;
//                     q.y = SEq_3;
//                     q.z = SEq_4;
                     q.w = q0;
                     q.x = q1;
                     q.y = q2;
                     q.z = q3;
                 }
                 else {
                     q = m.attitude.quaternion;
                 }
                 
                 float theta_rad = 2.0 * acos(q.w);
                 float theta_deg = theta_rad * (180.0 / M_PI);
                 float sin_theta_2 = sin (theta_rad / 2.0);
                 float qx = q.x / sin_theta_2;
                 float qy = q.y / sin_theta_2;
                 float qz = q.z / sin_theta_2;
                 
                 float vertacc = [self getVerticalAcceleration];
                 
                 // Speed, slowed by friction
                 float friction = 0.02;
                 _vspeed += vertacc;
                 friction *= SIGN (_vspeed);
                 if (ABS(friction) > ABS(_vspeed)) { friction = _vspeed; }
                 _vspeed -= friction;
                 
                 // Displacement, with magnet at 0.
                 float magnet = 0.075;
                 _vdisp += _vspeed;
                 magnet *= SIGN (_vdisp);
                 if (ABS(magnet) > ABS(_vdisp)) { magnet = _vdisp; }
                 _vdisp -= magnet;
                 
                 
                 //if (ABS(vertacc) > 0.1) {
                 if (0 && i % 5 == 0) {
                     NSLog(@"A:%d V:%d S:%d"
                           ,ROUND(100*vertacc)
                           ,ROUND(100*_vspeed)
                           ,ROUND(100*_vdisp));
                 }
                 
                 if (i % FREQ == 0) {
                     _lbx.text = nsprintf (@"%.2f", qx);
                     _lby.text = nsprintf (@"%.2f", qy);
                     _lbz.text = nsprintf (@"%.2f", qz);
                     _lbAngle.text = nsprintf (@"%.0f", theta_deg);
                     
                     _lbUsrAccX.text = nsprintf (@"%.2f", _userAcc.x);
                     _lbUsrAccY.text = nsprintf (@"%.2f", _userAcc.y);
                     _lbUsrAccZ.text = nsprintf (@"%.2f", _userAcc.z);
                     
                     _lbGravX.text = nsprintf (@"%.2f", _gravity.x);
                     _lbGravY.text = nsprintf (@"%.2f", _gravity.y);
                     _lbGravZ.text = nsprintf (@"%.2f", _gravity.z);
                     
                     _lbTotX.text = nsprintf (@"%.2f", totAcc.x);
                     _lbTotY.text = nsprintf (@"%.2f", totAcc.y);
                     _lbTotZ.text = nsprintf (@"%.2f", totAcc.z);
                 }

                 float x0,y0,z0;
                 if (_mode == USE_ACCELEROMETER) {
                     // Total accelerometer (dirty)
                     x0 = totAcc.x;
                     y0 = totAcc.y;
                     z0 = totAcc.z;
                 } else {
                     // Motion filtered out, clean attitude
                     x0 = _gravity.x;
                     y0 = _gravity.y;
                     z0 = _gravity.z;
                 }
                 
                 float x,y,z;
                 x=x0; y=z0; z=y0;
                 
#define BOUND(x,a,b) { if ((x) > (b)) (x) = (b); if ((x) < (a)) (x) = (a); }
                 BOUND(x,-1,1); BOUND(y,-1,1); BOUND(z,-1,1);
                 
                 // Cross product gives rot axis
                 float x2 = 0; float y2 = 0; float z2 = -1;
                 float xc = y*z2 - y2*z;
                 float yc = z*x2 - z2*x;
                 float zc = x*y2 - x2*y;
                 // Norm it
                 float lenc = sqrt(xc*xc + yc*yc + zc*zc);
                 xc /= lenc;
                 yc /= lenc;
                 zc /= lenc;
                 float omega = -acos(z);
                 
                 if (i % FREQ == 0) {
                     // Quaternion from (x,y,z) gravity
                     _lbXA.text = nsprintf (@"%.2f", xc);
                     _lbYA.text = nsprintf (@"%.2f", yc);
                     _lbZA.text = nsprintf (@"%.2f", zc);
                     _lbAngleA.text = nsprintf (@"%.0f", DEG(omega));
                 }
                 
                 if (_isConnected && (i % (FREQ / 10) == 0)) {
                     NSString *msg;
                     if (_mode == USE_FUSION || _mode == USE_MADGWICK) {
                         msg = nsprintf (@"%.4f,%.4f,%.4f,%.4f\n",theta_rad,qx,qy,qz);
                     } else {
                         msg = nsprintf (@"%.4f,%.4f,%.4f,%.4f\n", omega, xc,yc,zc);
                     }
                     [self write2server:msg];
                 }
             }]; // addOperationWithBlock
         }]; // motionmanager
    } // if (motion available)
} // viewDidLoad()


//---------------------------------
- (float) getVerticalAcceleration //@@@
//---------------------------------
{
    float v_acc[3];
    float v_acc_rot[3];

    // Rotate gravity to (0,1,0)
    LP_matrix rot = [self rotUp];
//    // Verify that it is really so
//    v_acc[0] = _gravity.x;
//    v_acc[1] = _gravity.y;
//    v_acc[2] = _gravity.z;
//    matxvec (&rot, v_acc, v_acc_rot);
//    NSLog (@"rot grav:%f %f %f", v_acc_rot[0],v_acc_rot[1],v_acc_rot[2]);
    
    // Apply gravity direction to external acceleration
    v_acc[0] = _userAcc.x;
    v_acc[1] = _userAcc.y;
    v_acc[2] = _userAcc.z;
    matxvec (&rot, v_acc, v_acc_rot);
    return v_acc_rot[1];
} // getVerticalAcceleration


//-------------------
- (LP_matrix) rotUp
//-------------------
{
    LP_matrix res;
    LP_matrix zrot;
    LP_matrix xrot;
    float x = _gravity.x;
    float y = _gravity.y;
    float z = _gravity.z;
    
    if (ABS(x) < 0.001) { x = 0.001; }
    if (ABS(y) < 0.001) { y = 0.001; }
    float xyLength = sqrt(x*(float)x + y*(float)y);
    
    float zAngle = SIGN(x) * (xyLength==0?0:acos(y/xyLength));
    
    float vecLength = sqrt(x*(float)x + y*(float)y + z*(float)z);
    
    float xAngle = -SIGN(z) * acos (xyLength / vecLength);
    
    // Rotation around z axis
    zrot.m[0][0] = cos(zAngle); zrot.m[0][1] =-sin(zAngle); zrot.m[0][2] = 0;
    zrot.m[1][0] = sin(zAngle); zrot.m[1][1] = cos(zAngle); zrot.m[1][2] = 0;
    zrot.m[2][0] = 0;           zrot.m[2][1] = 0;           zrot.m[2][2] = 1;
    
    //    xAngle = 0; // No rotation around x
    // Rotation around x axis
    xrot.m[0][0] = 1; xrot.m[0][1] = 0;           xrot.m[0][2] = 0;
    xrot.m[1][0] = 0; xrot.m[1][1] = cos(xAngle); xrot.m[1][2] = -sin(xAngle);
    xrot.m[2][0] = 0; xrot.m[2][1] = sin(xAngle); xrot.m[2][2] = cos(xAngle);
    
    matmul (&xrot,&zrot,&res);
    return res;
} // rotUp

#pragma mark Button Callbacks

//-------------------
- (void) cbConnect
//-------------------
{
    [self disconnectServer];

    if ([_txtIPA.text length] == 0
        || [_txtIPB.text length] == 0
        || [_txtIPC.text length] == 0
        || [_txtIPD.text length] == 0)
    {
        [self popup:@"IP part missing" title:@"Error"]; return;
    }
    if ([_txtPort.text length] == 0) {
        [self popup:@"Port missing" title:@"Error"]; return;
    }
    int ipa = [_txtIPA.text intValue];
    int ipb = [_txtIPB.text intValue];
    int ipc = [_txtIPC.text intValue];
    int ipd = [_txtIPD.text intValue];
    int port = [_txtPort.text intValue];
    if (ipa > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipb > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipc > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (ipd > 255) { [self popup:@"illegal IP" title:@"Error"]; return; }
    if (port > 0xffff) { [self popup:@"illegal Port" title:@"Error"]; return; }
    
    [self putNum:@"IPA" val:@(ipa)];
    [self putNum:@"IPB" val:@(ipb)];
    [self putNum:@"IPC" val:@(ipc)];
    [self putNum:@"IPD" val:@(ipd)];
    [self putNum:@"PORT" val:@(port)];
    
    NSString *server = nsprintf (@"%d.%d.%d.%d",ipa,ipb,ipc,ipd);
    [self connect2server:server port:port];
        //[self write2server:@"start_stream"];
    [self ledAmber];
} // cbConnect()

//----------------------------------
- (IBAction)btnConnect:(id)sender
//----------------------------------
{
    if (!_isConnected) {
        [self cbConnect];
    } else {
        [self disconnectServer];
    }
}

//-----------------------------
- (IBAction)btnLed:(id)sender
//-----------------------------
{
    [self btnConnect:sender];
}

//-------------------------------------
- (IBAction)btnLEDFusion:(id)sender
//-------------------------------------
{
    [self ledFusion];
}

//-------------------------------------
- (IBAction)btnLEDGravity:(id)sender
//-------------------------------------
{
    [self ledGravity];
}

//-------------------------------------
- (IBAction)btnLEDAccelerometer:(id)sender
//-------------------------------------
{
    [self ledAccelerometer];
}

//-----------------------------------------
- (IBAction)btnLEDMadgwick:(id)sender
//-----------------------------------------
{
    [self ledMadgwick];
}

#pragma mark IP Address

//----------------------------------------------------------
- (BOOL)               textField:(UITextField *)textField
   shouldChangeCharactersInRange:(NSRange)range
               replacementString:(NSString *)str
//----------------------------------------------------------
// Restrict all textfield input to digits only
{
    if (!strmatch(str,@"^[0-9]*$")) { return NO; }
    int maxlen = 3;
    if (textField == _txtPort) { maxlen = 5; }
    long len = [textField.text length] + [str length] - range.length;
    return (len > maxlen) ? NO : YES;
}

//-----------------
- (void) initIP
//-----------------
// Init textfields to values stored in userdefaults
{
    _txtIPA.text = [self getStr:@"IPA"];
    _txtIPB.text = [self getStr:@"IPB"];
    _txtIPC.text = [self getStr:@"IPC"];
    _txtIPD.text = [self getStr:@"IPD"];
    _txtPort.text = [self getStr:@"PORT"];
}

#pragma  mark Server Connectivity

//-------------------------------------------------------------
- (void)connectTimeout:(NSTimer*)timer
//-------------------------------------------------------------
// Socket connection timeout
{
    [self disconnectServer];
    [self popup:@"Connect failed" title:@"Error"];
}


// Make a TCP/IP connection to a server.
// Sets the member variables mIStream and mOStream.
// Use them to communicatte with the server.
//----------------------------------------
- (BOOL)connect2server:(NSString *)server
                  port:(int)port
//----------------------------------------
{
    
    // Give up after some seconds
    _connectTimeout =
    [NSTimer scheduledTimerWithTimeInterval: 4.0
                                     target: self
                                   selector: @selector(connectTimeout:)
                                   userInfo: nil
                                    repeats: NO];
    
    CFStreamCreatePairWithSocketToHost(kCFAllocatorDefault,
                                       (__bridge CFStringRef) server,
                                       port,
                                       &mReadStream,
                                       &mWriteStream);
    
    if (mReadStream && mWriteStream) {
        CFReadStreamSetProperty (mReadStream,
                                kCFStreamPropertyShouldCloseNativeSocket,
                                kCFBooleanTrue);
        CFWriteStreamSetProperty (mWriteStream,
                                 kCFStreamPropertyShouldCloseNativeSocket,
                                 kCFBooleanTrue);
        
        mIStream = (__bridge NSInputStream *)mReadStream;
        //[mIStream retain];
        [mIStream setDelegate:self];
        [mIStream scheduleInRunLoop:[NSRunLoop mainRunLoop]
                            forMode:NSDefaultRunLoopMode];
        [mIStream open];
        
        mOStream = (__bridge NSOutputStream *)mWriteStream;
        //[mOStream retain];
        [mOStream setDelegate:self];
        [mOStream scheduleInRunLoop:[NSRunLoop mainRunLoop]
                            forMode:NSDefaultRunLoopMode];
        [mOStream open];
        
        return YES;
    }
    return NO;
} // connect2server()

//--------------------------
- (void) ledGreen
//--------------------------
{
    UIImage *greenLED = [UIImage imageNamed:@"green-led-on-md.png"];
    [_btnLED setImage:greenLED forState:UIControlStateNormal];
    _isConnected = YES;
}

//--------------------------
- (void) ledRed
//--------------------------
{
    UIImage *redLED = [UIImage imageNamed:@"led-red-control-md.png"];
    [_btnLED setImage:redLED forState:UIControlStateNormal];
    _isConnected = NO;
}

//--------------------------
- (void) ledAmber
//--------------------------
{
    UIImage *amberLED = [UIImage imageNamed:@"amber-led-on-md.png"];
    [_btnLED setImage:amberLED forState:UIControlStateNormal];
    _isConnected = NO;
}

//--------------------
- (void) ledFusion
//--------------------
{
    UIImage *blackLED = [UIImage imageNamed:@"black-led-off-md.png"];
    UIImage *grayLED = [UIImage imageNamed:@"grey-led-md.png"];
    [_btnLEDFusion setImage:blackLED forState:UIControlStateNormal];
    [_btnLEDGravity setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDAccelerometer setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDMadgwick setImage:grayLED forState:UIControlStateNormal];
    _mode = USE_FUSION;
}

//--------------------
- (void) ledMadgwick
//--------------------
{
    UIImage *blackLED = [UIImage imageNamed:@"black-led-off-md.png"];
    UIImage *grayLED = [UIImage imageNamed:@"grey-led-md.png"];
    [_btnLEDFusion setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDGravity setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDAccelerometer setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDMadgwick setImage:blackLED forState:UIControlStateNormal];
    _mode = USE_MADGWICK;
}

//--------------------
- (void) ledGravity
//--------------------
{
    UIImage *blackLED = [UIImage imageNamed:@"black-led-off-md.png"];
    UIImage *grayLED = [UIImage imageNamed:@"grey-led-md.png"];
    [_btnLEDFusion setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDGravity setImage:blackLED forState:UIControlStateNormal];
    [_btnLEDAccelerometer setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDMadgwick setImage:grayLED forState:UIControlStateNormal];
    _mode = USE_GRAVITY;
}
//-------------------------
- (void) ledAccelerometer
//-------------------------
{
    UIImage *blackLED = [UIImage imageNamed:@"black-led-off-md.png"];
    UIImage *grayLED = [UIImage imageNamed:@"grey-led-md.png"];
    [_btnLEDFusion setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDGravity setImage:grayLED forState:UIControlStateNormal];
    [_btnLEDAccelerometer setImage:blackLED forState:UIControlStateNormal];
    [_btnLEDMadgwick setImage:grayLED forState:UIControlStateNormal];
    _mode = USE_ACCELEROMETER;
}

// Disconnect from server
//--------------------------
- (void)disconnectServer
//--------------------------
{
    if (mIStream != nil) [mIStream close];
    if (mOStream != nil) [mOStream close];
    mIStream = nil;
    mOStream = nil;
    [self ledRed];
} // disconnectServer()

// Write a null terminated string to the server
//-------------------------------------------
-(void) write2server:(NSString *)p_msg
//-------------------------------------------
{
    const char *msg = [p_msg cStringUsingEncoding:NSUTF8StringEncoding];
    [mOStream write:(uint8_t *)msg maxLength:strlen(msg)];
} // write2server()

// Stream events and data from the server
//------------------------------------------------------------------------
- (void)stream:(NSStream *)stream handleEvent:(NSStreamEvent)eventCode
//------------------------------------------------------------------------
{
    if (_connectTimeout) {
        [_connectTimeout invalidate];
        _connectTimeout = nil;
    }
    NSMutableData *data = [NSMutableData new];
    switch(eventCode) {
        case NSStreamEventHasBytesAvailable: {
            uint8_t buf[1024];
            unsigned long len = 0;
            len = [(NSInputStream *)stream read:buf maxLength:1024];
            if(len) {
                [data appendBytes:(const void *)buf length:len];
                int bytesRead;
                bytesRead += len;
            } else {
                NSLog(@"No data.");
            }
            NSString *str =
            [[NSString alloc] initWithData:data
                                  encoding:NSUTF8StringEncoding];
            if ([str length]) {
                [self popup:str title:@"Server Message"];
            }
            break;
        }
        case NSStreamEventErrorOccurred: {
            [self disconnectServer];
            NSError *theError = [stream streamError];
            NSString *msg = nsprintf (@"Error %d: %@"
                                      ,[theError code]
                                      ,[theError localizedDescription]);
            [self popup:msg title: @"Stream Error"];
            break;
        }
        case NSStreamEventNone: {
            NSLog (@"NSStreamEventNone");
            break;
        }
        case NSStreamEventOpenCompleted: {
            [self ledGreen];
            NSLog (@"NSStreamEventOpenCompleted");
            break;
        }
        case NSStreamEventHasSpaceAvailable: {
            //NSLog (@"NSStreamEventHasSpaceAvailable");
            break;
        }
        case NSStreamEventEndEncountered: {
            NSLog (@"NSStreamEventEndEncountered");
            break;
        }
        default: {
            [self popup:@"unknown stream event" title: @"Stream Error"];
        }
    } // switch
} // handleEvent

#pragma mark UI Helpers

//-------------------------------
- (void) popup:(NSString *)msg
         title:(NSString *)title
//-------------------------------
{
    UIAlertView *alert =
    [[UIAlertView alloc] initWithTitle:title
                               message:msg
                              delegate:self
                     cancelButtonTitle:@"OK"
                     otherButtonTitles:nil];
    [alert show];
}

#pragma mark Userdefaults

#define DEF [NSUserDefaults standardUserDefaults]

//-----------------------------------------------------
- (void) putNum:(NSString *)key val:(NSNumber *)val
//-----------------------------------------------------
// Store a number in UserDefaults
{
    [DEF setObject:val forKey:key];
}

//-----------------------------------------------------
- (NSNumber *) getNum:(NSString *)key
//-----------------------------------------------------
// Get number from UserDefaults
{
    return [DEF objectForKey:key];
}

//-----------------------------------------------------
- (int) getInt:(NSString *)key
//-----------------------------------------------------
// Get number from UserDefaults, return as int
{
    return [[DEF objectForKey:key] intValue];
}

//-----------------------------------------------------
- (NSString *) getStr:(NSString *)key
//-----------------------------------------------------
// Get object from UserDefaults, return as string
{
    id obj = [DEF objectForKey:key];
    return obj ? nsprintf (@"%@", [DEF objectForKey:key]) : @"" ;
}

#pragma  mark C Funcs

#define RLOOP(N) for(r=0;r<(N);r++)
#define CLOOP(N) for(c=0;c<(N);c++)
#define SIGN(x) ((x)>=0?1:-1)


//---------------------------------
CMQuaternion rot2quat (LP_matrix *p_m)
//---------------------------------
// Make a quaternion from a rot matrix.
// This is a little messy for numeric stability.
// (by Martin Baker on euklideanspace.com)
{
    float (*m)[3];
    m = p_m->m; //  m[1][2] == p_m->m[1][2]
    CMQuaternion res;
    
    float tr = m[0][0] + m[1][1] + m[2][2];
    
    if (tr > 0) {
        float S = sqrt(tr+1.0) * 2; // S=4*qw
        res.w = 0.25 * S;
        res.x = (m[2][1] - m[1][2]) / S;
        res.y = (m[0][2] - m[2][0]) / S;
        res.z = (m[1][0] - m[0][1]) / S;
    } else if ((m[0][0] > m[1][1])&(m[0][0] > m[2][2])) {
        float S = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx
        res.w = (m[2][1] - m[1][2]) / S;
        res.x = 0.25 * S;
        res.y = (m[0][1] + m[1][0]) / S;
        res.z = (m[0][2] + m[2][0]) / S;
    } else if (m[1][1] > m[2][2]) {
        float S = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
        res.w = (m[0][2] - m[2][0]) / S;
        res.x = (m[0][1] + m[1][0]) / S;
        res.y = 0.25 * S;
        res.z = (m[1][2] + m[2][1]) / S;
    } else {
        float S = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
        res.w = (m[1][0] - m[0][1]) / S;
        res.x = (m[0][2] + m[2][0]) / S;
        res.y = (m[1][2] + m[2][1]) / S;
        res.z = 0.25 * S;
    }
    return res;
} // rot2quat

// Transpose a matrix
//-----------------------------------------
void transpose (LP_matrix *p_m, LP_matrix *p_res)
//-----------------------------------------
{
    if(p_m == NULL) return;
    
    int r,c;
    RLOOP(3) {
        CLOOP(3) {
            p_res->m[c][r] = p_m->m[r][c];
        }
    }
} // transpose()

// Multiply two 3x3 matrices
//-----------------------------------------------------
void matmul (LP_matrix *p_m1, LP_matrix *p_m2, LP_matrix *p_res)
//-----------------------------------------------------
{
    if(p_m1 == NULL || p_m2 == NULL) return;
    
    int r,c;
    LP_matrix m2;
    transpose(p_m2,&m2);
    RLOOP(3) {
        CLOOP(3) {
            float *r1 = p_m1->m[r];
            float *r2 = m2.m[c];
            p_res->m[r][c] = r1[0]*r2[0] + r1[1]*r2[1] + r1[2]*r2[2];
        } // CLOOP
    } // RLOOP
} // matmul()

// Multiply a vector and a matrix (3D)
//---------------------------------------------------------
void matxvec (LP_matrix *p_m, float *p_v, float *p_res)
//---------------------------------------------------------
{
    if(p_m == NULL) return;
    
    int r;
    RLOOP(3) {
        float *row = p_m->m[r];
        p_res[r] = (row[0]*p_v[0] + row[1]*p_v[1] + row[2]*p_v[2]);
    } // RLOOP
} // matxvec()


// Return a pointer to a rotation matrix aligning a vector
// with the y axis.
//-----------------------------------------------
LP_matrix *alignVector (float x, float y,float z)
//-----------------------------------------------
{
    static LP_matrix res;
    LP_matrix zrot;
    LP_matrix xrot;
    if (ABS(x) < 0.001) { x = 0.001; }
    if (ABS(y) < 0.001) { y = 0.001; }
    float xyLength = sqrt(x*(float)x + y*(float)y);
    
    float zAngle = SIGN(x) * (xyLength==0?0:acos(y/xyLength));
    
    float vecLength = sqrt(x*(float)x + y*(float)y + z*(float)z);
    
    float xAngle = -SIGN(z) * acos (xyLength / vecLength);
    
    // Rotation around z axis
    zrot.m[0][0] = cos(zAngle); zrot.m[0][1] =-sin(zAngle); zrot.m[0][2] = 0;
    zrot.m[1][0] = sin(zAngle); zrot.m[1][1] = cos(zAngle); zrot.m[1][2] = 0;
    zrot.m[2][0] = 0;           zrot.m[2][1] = 0;           zrot.m[2][2] = 1;
    
    // Rotation around x axis
    xrot.m[0][0] = 1; xrot.m[0][1] = 0;           xrot.m[0][2] = 0;
    xrot.m[1][0] = 0; xrot.m[1][1] = cos(xAngle); xrot.m[1][2] = -sin(xAngle);
    xrot.m[2][0] = 0; xrot.m[2][1] = sin(xAngle); xrot.m[2][2] = cos(xAngle);
    
    matmul (&xrot,&zrot,&res);
    return &res;
} // alignVector()

//------------------------------------------
NSString *nsprintf (NSString *format, ...)
//------------------------------------------
{
    va_list args;
    va_start(args, format);
    NSString *msg =[[NSString alloc] initWithFormat:format
                                          arguments:args];
    return msg;
} // nsprintf()

//---------------------------------------------
BOOL strmatch (NSString *str, NSString *pat)
//---------------------------------------------
// Check whether string matches regex
{
    NSRegularExpression *re =
    [NSRegularExpression regularExpressionWithPattern:pat options:0 error:NULL];
    NSTextCheckingResult *match =
    [re firstMatchInString:str options:0 range:NSMakeRange(0, [str length])];
    return [match numberOfRanges]?YES:NO;
}

#pragma Cruft

//--------------------------------------------------------
-(BOOL) textFieldShouldReturn:(UITextField *)textField
//--------------------------------------------------------
// Hide keyboards on return
{
    [textField resignFirstResponder];
    return YES;
}

//// Madgwick from paper
////-------------------------
//
//// System constants
//// #define deltat 0.001f                                     // sampling period in seconds (shown as 1 ms)
//#define deltat (1.0 / FREQ)
//#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f)  // gyroscope measurement error in rad/s (shown as 5 deg/s)
//#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f)  // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
//#define beta sqrt(3.0f / 4.0f) * gyroMeasError            // compute beta
//#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift            // compute zeta
//
//// Global system variables
//float a_x, a_y, a_z;                                   // accelerometer measurements
//float w_x, w_y, w_z;                                   // gyroscope measurements in rad/s
//float m_x, m_y, m_z;                                   // magnetometer measurements
//float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0;      // estimated orientation quaternion elements with initial conditions
//float b_x = 1, b_z = 0;                                // reference direction of flux in earth frame
//float w_bx = 0, w_by = 0, w_bz = 0;                    // estimate gyroscope biases error
//
//// Madwick sensor fusion
////----------------------------------------------------------
//void filterUpdate (float w_x, float w_y, float w_z
//                   ,float a_x, float a_y, float a_z
//                   ,float m_x, float m_y, float m_z)
////----------------------------------------------------------
//{
//    
//    if (m_x == 0 && m_y == 0 && m_z == 0) return; // AHN
//    
//    // local system variables
//    float norm;                                                             // vector norm
//    float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;   // quaternion rate from gyroscopes elements
//    float f_1, f_2, f_3, f_4, f_5, f_6;                                     // objective function elements
//    float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,               // objective function Jacobian elements
//    J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
//    float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;               // estimated direction of the gyroscope error
//    float w_err_x, w_err_y, w_err_z;                                        // estimated direction of the gyroscope error (angular)
//    float h_x, h_y, h_z;                                                    // computed flux in the earth frame
//    // axulirary variables to avoid reapeated calcualtions
//    float halfSEq_1 = 0.5f * SEq_1;
//    float halfSEq_2 = 0.5f * SEq_2;
//    float halfSEq_3 = 0.5f * SEq_3;
//    float halfSEq_4 = 0.5f * SEq_4;
//    float twoSEq_1 = 2.0f * SEq_1;
//    float twoSEq_2 = 2.0f * SEq_2;
//    float twoSEq_3 = 2.0f * SEq_3;
//    float twoSEq_4 = 2.0f * SEq_4;
//    float twob_x = 2.0f * b_x;
//    float twob_z = 2.0f * b_z;
//    float twob_xSEq_1 = 2.0f * b_x * SEq_1;
//    float twob_xSEq_2 = 2.0f * b_x * SEq_2;
//    float twob_xSEq_3 = 2.0f * b_x * SEq_3;
//    float twob_xSEq_4 = 2.0f * b_x * SEq_4;
//    float twob_zSEq_1 = 2.0f * b_z * SEq_1;
//    float twob_zSEq_2 = 2.0f * b_z * SEq_2;
//    float twob_zSEq_3 = 2.0f * b_z * SEq_3;
//    float twob_zSEq_4 = 2.0f * b_z * SEq_4;
//    float SEq_1SEq_2;
//    float SEq_1SEq_3 = SEq_1 * SEq_3;
//    float SEq_1SEq_4;
//    float SEq_2SEq_3;
//    float SEq_2SEq_4 = SEq_2 * SEq_4;
//    float SEq_3SEq_4;
//    float twom_x = 2.0f * m_x;
//    float twom_y = 2.0f * m_y;
//    float twom_z = 2.0f * m_z;
//    // normalise the accelerometer measurement
//    norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
//    a_x /= norm;
//    a_y /= norm;
//    a_z /= norm;
//    // normalise the magnetometer measurement
//    norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
//    m_x /= norm;
//    m_y /= norm;
//    m_z /= norm;
//    // compute the objective function and Jacobian
//    f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
//    f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
//    f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
//    f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
//    f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
//    f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
//    J_11or24 = twoSEq_3;                                                    // J_11 negated in matrix multiplication
//    J_12or23 = 2.0f * SEq_4;
//    J_13or22 = twoSEq_1;                                                    // J_12 negated in matrix multiplication
//    J_14or21 = twoSEq_2;
//    J_32 = 2.0f * J_14or21;                                                 // negated in matrix multiplication
//    J_33 = 2.0f * J_11or24;                                                 // negated in matrix multiplication
//    J_41 = twob_zSEq_3;                                                     // negated in matrix multiplication
//    J_42 = twob_zSEq_4;
//    J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1;                                // negated in matrix multiplication
//    J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2;                                // negated in matrix multiplication
//    J_51 = twob_xSEq_4 - twob_zSEq_2;                                       // negated in matrix multiplication
//    J_52 = twob_xSEq_3 + twob_zSEq_1;
//    J_53 = twob_xSEq_2 + twob_zSEq_4;
//    J_54 = twob_xSEq_1 - twob_zSEq_3;                                       // negated in matrix multiplication
//    J_61 = twob_xSEq_3;
//    J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
//    J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
//    J_64 = twob_xSEq_2;
//    // compute the gradient (matrix multiplication)
//    SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
//    SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
//    SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
//    SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
//    // normalise the gradient to estimate direction of the gyroscope error
//    norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
//    SEqHatDot_1 = SEqHatDot_1 / norm;
//    SEqHatDot_2 = SEqHatDot_2 / norm;
//    SEqHatDot_3 = SEqHatDot_3 / norm;
//    SEqHatDot_4 = SEqHatDot_4 / norm;
//    // compute angular estimated direction of the gyroscope error
//    w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
//    w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
//    w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
//    // compute and remove the gyroscope baises
//    w_bx += w_err_x * deltat * zeta;
//    w_by += w_err_y * deltat * zeta;
//    w_bz += w_err_z * deltat * zeta;
//    w_x -= w_bx;
//    w_y -= w_by;
//    w_z -= w_bz;
//    // compute the quaternion rate measured by gyroscopes
//    SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
//    SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
//    SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
//    SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
//    // compute then integrate the estimated quaternion rate
//    SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
//    SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
//    SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
//    SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
//    // normalise quaternion
//    norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
//    SEq_1 /= norm;
//    SEq_2 /= norm;
//    SEq_3 /= norm;
//    SEq_4 /= norm;
//    // compute flux in the earth frame
//    SEq_1SEq_2 = SEq_1 * SEq_2;                                             // recompute axulirary variables
//    SEq_1SEq_3 = SEq_1 * SEq_3;
//    SEq_1SEq_4 = SEq_1 * SEq_4;
//    SEq_3SEq_4 = SEq_3 * SEq_4;
//    SEq_2SEq_3 = SEq_2 * SEq_3;
//    SEq_2SEq_4 = SEq_2 * SEq_4;
//    h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
//    h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
//    h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
//    // normalise the flux vector to have only components in the x and z
//    b_x = sqrt((h_x * h_x) + (h_y * h_y));
//    b_z = h_z;
//}

#define sampleFreq	((float)FREQ)		// sample frequency in Hz

// Madgwick downloaded
//----------------------
//
//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0f		// sample frequency in Hz
//#define betaDef		0.1f		// 2 * proportional gain
#define betaDef		0.5f		// 2 * proportional gain
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)


//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        
        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;
        
        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
    
    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;
        
        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;
        
        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }
    
    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}


// Mahoney downloaded
//----------------------

//---------------------------------------------------------------------------------------------------
// Definitions

//#define sampleFreq	512.0f			// sample frequency in Hz
#define sampleFreq	((float)FREQ)
//#define twoKpDef	(2.0f * 0.5f)	// 2 * proportional gain
#define twoKpDef	(2.0f * 0.0001f)	// 2 * proportional gain
//#define twoKiDef	(2.0f * 0.0f)	// 2 * integral gain
#define twoKiDef	(2.0f * 0.0001f)	// 2 * integral gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float twoKp = twoKpDef;											// 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;											// 2 * integral gain (Ki)
//volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;					// quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f,  integralFBy = 0.0f, integralFBz = 0.0f;	// integral error terms scaled by Ki

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
    float recipNorm;
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    float hx, hy, bx, bz;
    float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);
        return;
    }
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;
        
        // Auxiliary variables to avoid repeated arithmetic
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;
        
        // Reference direction of Earth's magnetic field
        hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
        hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
        bx = sqrt(hx * hx + hy * hy);
        bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));
        
        // Estimated direction of gravity and magnetic field
        halfvx = q1q3 - q0q2;
        halfvy = q0q1 + q2q3;
        halfvz = q0q0 - 0.5f + q3q3;
        halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
        halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
        halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);
        
        // Error is sum of cross product between estimated direction and measured direction of field vectors
        halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
        halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
        halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);
        
        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;
    
    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;
        
        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;
        
        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);
        
        // Compute and apply integral feedback if enabled
        if(twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);	// integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        }
        else {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }
        
        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }
    
    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));		// pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx); 
    
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
}

float invSqrt(float number) {
    union {
        float f;
        int32_t i;
    } y;
    
    y.f = number;
    y.i = 0x5f375a86 - (y.i >> 1);
    y.f = y.f * ( 1.5f - ( number * 0.5f * y.f * y.f ) );
    return y.f;
}

#if 0

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

#endif

@end



