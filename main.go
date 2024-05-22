package main

import (
	"flag"
	"fmt"
	"image"
	"image/color"
	"math"
	"strconv"
	"strings"

	"github.com/brunoga/robomaster"
	"github.com/brunoga/robomaster-tracker/mode"
	"github.com/brunoga/robomaster/module/camera"
	"github.com/brunoga/robomaster/module/chassis"
	"github.com/brunoga/robomaster/module/chassis/controller"
	"github.com/brunoga/robomaster/module/robot"
	"github.com/brunoga/robomaster/support/pid"
	"github.com/faiface/mainthread"
	"gocv.io/x/gocv"
)

// Flags.
var (
	hsvLower = flag.String("hsvlower", "35,219,90",
		"lower bound for color filtering (h,s,v)")
	hsvUpper = flag.String("hsvupper", "119,255,255",
		"lower bound for color filtering (h,s,v)")
)

type exampleVideoHandler struct {
	window        *gocv.Window
	tracker       *mode.ColorObject
	chassisModule *chassis.Chassis
	pidPitch      pid.Controller
	pidYaw        pid.Controller
	quitChan      chan struct{}
}

func parseHSVValues(hsvString string) (float64, float64, float64, error) {
	components := strings.Split(hsvString, ",")
	if len(components) != 3 {
		return -1, -1, -1, fmt.Errorf("invalid hsv values")
	}

	h, err := strconv.Atoi(components[0])
	if err != nil {
		return -1, -1, -1, fmt.Errorf("invalid h value")
	}

	s, err := strconv.Atoi(components[1])
	if err != nil {
		return -1, -1, -1, fmt.Errorf("invalid s value")
	}

	v, err := strconv.Atoi(components[2])
	if err != nil {
		return -1, -1, -1, fmt.Errorf("invalid v value")
	}

	return float64(h), float64(s), float64(v), nil
}

func newExampleVideoHandler(
	chassisModule *chassis.Chassis) (*exampleVideoHandler, error) {

	var window *gocv.Window
	mainthread.Call(func() {
		window = gocv.NewWindow("Robomaster")
		window.ResizeWindow(camera.HorizontalResolutionPoints/2,
			camera.VerticalResolutionPoints/2)
	})

	hl, sl, vl, err := parseHSVValues(*hsvLower)
	if err != nil {
		return nil, err
	}

	hu, su, vu, err := parseHSVValues(*hsvUpper)
	if err != nil {
		return nil, err
	}

	return &exampleVideoHandler{
		window,
		mode.NewColorObject(hl, sl, vl, hu, su, vu, 10),
		chassisModule,

		// TODO(bga): Tune these values.
		pid.NewPIDController(0.7, 0.0, 0.0, -1, 1),
		pid.NewPIDController(0.7, 0.0, 0.0, -1, 1),

		make(chan struct{}),
	}, nil
}

func (e *exampleVideoHandler) QuitChan() <-chan struct{} {
	return e.quitChan
}

func (e *exampleVideoHandler) HandleFrame(frame *camera.RGB) {
	// Do not explicitly close inFrameRGBA as the underlying pixel data is
	// managed by Go itself.
	inFrameRGBA, err := gocv.NewMatFromBytes(camera.VerticalResolutionPoints,
		camera.HorizontalResolutionPoints, gocv.MatTypeCV8UC3, frame.Pix)
	if err != nil {
		return
	}

	// A Mat underlying pixel format is BRG, so we convert to it. Note we use
	// ColorBGRToRGB as the color conversion code as there is no ColorRGBToBGR
	// (which is fine as the conversion works both ways anyway).
	inFrame := gocv.NewMatWithSize(720, 1280, gocv.MatTypeCV8SC3)
	gocv.CvtColor(inFrameRGBA, &inFrame, gocv.ColorBGRToRGB)
	defer inFrame.Close()

	// Clone frame as we are going to modify it. We could potentially call
	// wg.Done() right after this but without implementing a queue, this is not
	// a good idea (frames will be racing against each other).
	outputFrame := inFrame.Clone()
	defer outputFrame.Close()

	x, y, radius, err := e.tracker.FindLargestObject(&inFrame)
	if err == nil {
		// Found something. Draw a circle around it into our modified frame.
		gocv.Circle(&outputFrame, image.Point{X: int(x), Y: int(y)},
			int(radius), color.RGBA{R: 0, G: 255, B: 255, A: 255}, 2)

		// Get errors in the x and y axis normalized to [-0.5, 0.5]
		errX := float64(x-(camera.HorizontalResolutionPoints/2)) /
			camera.HorizontalResolutionPoints
		errY := float64((camera.VerticalResolutionPoints/2)-y) /
			camera.VerticalResolutionPoints

		fmt.Println("errX: ", errX, "errY: ", errY)

		// If there is some error (object in not in center of image), move the
		// gimbal to minimize it.
		if math.Abs(errX) > 0.0 || math.Abs(errY) > 0.0 {
			// Move the gimbal with a speed determined by the pitch and yaw PID
			// controllers.
			outputX := e.pidYaw.Output(errX)
			outputY := e.pidPitch.Output(errY)
			fmt.Printf("x: %f, y: %f\n", outputX, outputY)
			gimbalStickPosition := controller.StickPosition{
				X: outputX,
				Y: -outputY,
			}
			err = e.chassisModule.Move(nil, &gimbalStickPosition, controller.ModeSDK)
			if err != nil {
				fmt.Println(err)
			}
		}
	}

	// Show output frame and wait for an event.
	mainthread.Call(func() {
		e.window.IMShow(outputFrame)
		e.window.WaitKey(1)
	})

	// Hack to detect that the window was closed.
	//if e.window.GetWindowProperty(gocv.WindowPropertyAspectRatio) == -1.0 {
	// Window closed. Notify listeners.
	//close(e.quitChan)
	//}
}

func run() {
	client, err := robomaster.New(nil, 0)
	if err != nil {
		panic(err)
	}

	err = client.Start()
	if err != nil {
		panic(err)
	}
	defer client.Stop()

	// Obtain references to the modules we are interested in.
	chassisModule := client.Chassis()
	cameraModule := client.Camera()

	err = chassisModule.SetMode(chassis.ModeFPV)
	if err != nil {
		panic(err)
	}

	err = client.Robot().SetChassisSpeedLevel(robot.ChassisSpeedLevelFast)
	if err != nil {
		panic(err)
	}

	videoHandler, err := newExampleVideoHandler(chassisModule)
	if err != nil {
		panic(err)
	}

	token, err := cameraModule.AddVideoCallback(videoHandler.HandleFrame)
	if err != nil {
		panic(err)
	}
	defer cameraModule.RemoveVideoCallback(token)

	<-videoHandler.QuitChan()
}

func main() {
	mainthread.Run(run)
}
