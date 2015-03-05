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
@property enum {USE_FUSION, USE_GRAVITY, USE_ACCELEROMETER} mode;

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
        const int FREQ = 25;
        _motionManager.deviceMotionUpdateInterval = 1.0 / FREQ;
        [self.motionManager
         startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryZVertical
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXArbitraryCorrectedZVertical
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXMagneticNorthZVertical
         //startDeviceMotionUpdatesUsingReferenceFrame:CMAttitudeReferenceFrameXTrueNorthZVertical
         toQueue:self.deviceQueue
         withHandler:^(CMDeviceMotion *m, NSError *error)
         {
             [[NSOperationQueue mainQueue] addOperationWithBlock:^{
                 i++; 
                 CMQuaternion q = m.attitude.quaternion;
                 float theta_rad = 2.0 * acos(q.w);
                 float theta_deg = theta_rad * (180.0 / M_PI);
                 float sin_theta_2 = sin (theta_rad / 2.0);
                 float qx = q.x / sin_theta_2;
                 float qy = q.y / sin_theta_2;
                 float qz = q.z / sin_theta_2;
                 // Norm the vector to 1.0
                 //float vlen = sqrt (x*x+y*y+z*z);
                 //x /= vlen; y /= vlen; z /= vlen;
                 
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
                 if (i % 5 == 0) {
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
                 
                 if (_isConnected) {
                     NSString *msg;
                     if (_mode == USE_FUSION) {
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
    _mode = USE_FUSION;
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


@end



