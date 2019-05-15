    package com.example.mwnl_msjang.light;

    import android.hardware.Sensor;
    import android.hardware.SensorEvent;
    import android.hardware.SensorEventListener;
    import android.hardware.SensorManager;
    import android.os.CountDownTimer;
    import android.os.Environment;
    import android.support.v7.app.AppCompatActivity;
    import android.os.Bundle;
    import android.util.Log;
    import android.view.View;
    import android.widget.Button;
    import android.widget.ImageView;
    import android.widget.TextView;

    import java.io.BufferedWriter;
    import java.io.File;
    import java.io.FileWriter;
    import java.io.IOException;
    import java.text.SimpleDateFormat;
    import java.util.ArrayList;
    import java.util.Date;
    import java.util.Iterator;
    import java.util.Stack;
    import java.util.Timer;
    import java.util.TimerTask;

    public class MainActivity extends AppCompatActivity {

        // PARAMETERS //

        double ANGLE_DETECTION_ACC_THRESHOLD = 0.2;
        int LCSS_THRESHOLD = 100;
        double STRAIGHT_TEST_LCSS_THRESHOLD = 0.1;
        double STAIR_TEST_LCSS_THRESHOLD = 0.3;
        double NAVIGATION_TURN_DETECTION_THRESHOLD_DRGREE = 20;

        ///////////////

        TextView t_light, t_gyro_x, Time;
        TextView t_inst_turn, t_inst_step_number;
        TextView t_simil;

        ImageView arrow_direction;

        Button saveme, testme, btnclear, memo_button;
        boolean isrunning=false;
        boolean istesting=false;
        Long StartTime;
        String Time_Sec;

        String dirPath;

        ArrayList<light_value> light_current_array;
        ArrayList<acc_value> acc_current_array;
        ArrayList<gyro_value> gyro_current_array;

        ArrayList<normal_trace> normal_trace_array;

        ArrayList<Long> LCS_TIME_array;

        ///////////////////////////////////////// 로그 저장용 날짜 관련

        long mNow;
        Date mDate;;
        SimpleDateFormat mFormatSec = new SimpleDateFormat("yyMMdd_HHmmss");

        /////////////////////////////////////////

        private Sensor accSensor, gyroSensor,  lightSensor;
        private SensorEventListener accLis, gyroLis, lightLis;
        private boolean stepDetectionFlag = true;

        private double xCoordinate = 0;
        private double yCoordinate = 0;

        private long sysTime;
        private int stepCount = 0;

        private int return_iter=0;

        private float aggRotation;
        private float filteredRotation;

        private static final double RAD2DGR = 180 / Math.PI;
        private static final float MS2S = 1.0f / 1000.0f;

        private float prevAcc;
        private float filteredAcc;
        private float rotation;

        private double filtered_acc=0;
        private double acc_tendency=0;
        private double light_tendency=0;
        private double pre_acc_value=0;


        private double previous_step_time=0;
        private int previous_step_count=0;

        private double filtered_light=0;
        private double previous_light_peak_time =0;
        private double pre_light_value = 0;

        private int light_peak_count=0;
        private int cur_test_peak_count=0;

        private int global_test_state=0;
        private int current_turn_flag=0;   // 추후 각으로 바꿀 필요가 있다.
        private int stair_flag=0;
        private double turn_flag=0;

        private int memo_button_pressed=0;
        private double gyro_tendency=0;
        private double pre_gyro_value=0;

        int second = 0;
        private Timer mTimer , test_timer;

        private boolean next_instruction_loaded=false;
        private boolean test_turn_detection=false;

        private SensorManager sensorManager;


        @Override
        protected void onCreate(Bundle savedInstanceState) {
            super.onCreate(savedInstanceState);
            setContentView(R.layout.activity_main);


            t_light = (TextView) findViewById(R.id.t_light);
            t_gyro_x = (TextView) findViewById(R.id.t_gyro_x);
            t_inst_turn = (TextView) findViewById(R.id.inst_turn);
            t_inst_step_number = (TextView) findViewById(R.id.inst_step_number);

            Time = (TextView) findViewById(R.id.time);
            t_simil = (TextView) findViewById(R.id.t_simil);

            saveme = (Button) findViewById(R.id.button_submit);
            testme = (Button) findViewById(R.id.button_test);
            memo_button  = (Button) findViewById(R.id.button_turn);

            btnclear = (Button) findViewById(R.id.button_clear);

            arrow_direction = (ImageView) findViewById(R.id.imageView1);

            sensorManager = (SensorManager) getSystemService(SENSOR_SERVICE);

            if (sensorManager != null) {
                accSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION);
                gyroSensor = sensorManager.getDefaultSensor(Sensor.TYPE_GYROSCOPE);
                lightSensor = sensorManager.getDefaultSensor(Sensor.TYPE_LIGHT);
            }

            test_timer = new Timer();


            mTimer = new Timer();
            mTimer.schedule(new OnesecTimer(), 0 , 1000);

            accLis = new accelerometerListener();
            gyroLis = new gyroscopeListener();
            lightLis = new lightmeterlistener();

            acc_current_array = new ArrayList<>();
            light_current_array = new ArrayList<>();
            gyro_current_array = new ArrayList<>();

            normal_trace_array = new ArrayList<>();

            LCS_TIME_array = new ArrayList<>();

            saveme.setOnClickListener(new Button.OnClickListener() {

                public void onClick(View v) {
                    try {
                        if (!isrunning) {
                            saveme.setBackgroundColor(0xFFFF0000);
                            saveme.setText("   STOP_SAVE   ");
                            StartTime = System.currentTimeMillis();
                            isrunning = true;

                            Time_Sec = getTimeSec();  // for saving files.

                            init_value();

                            return_iter=0;

                            mTimer.cancel();
                            mTimer = new Timer();
                            mTimer.schedule(new OnesecTimer(), 0 , 1000);

                            arrow_direction.setImageResource(R.drawable.navigation);


                        } else {
                            isrunning = false;
                            saveme.setBackgroundResource(android.R.drawable.btn_default);
                            saveme.setText("   SAVE_START   ");

                            gyro_array_degree_finder();

                            buftotxt("LIGHT","NORMAL");
                            buftotxt("ACC","NORMAL");
                            buftotxt("GYRO","NORMAL");

                            buftotxt("NORMAL_TRACE","NORMAL");

                            light_current_array.clear();
                            acc_current_array.clear();
                            gyro_current_array.clear();

                            second = 0;
                            mTimer.cancel();
                            mTimer = new Timer();
                            mTimer.schedule(new OnesecTimer(), 0 , 1000);

                        }
                    } catch (Exception ex) {
                        Time.setText("URL EXCEPTION");
                    }
                }
            });

            testme.setOnClickListener(new Button.OnClickListener() {


                public void onClick(View v){

                    if(!istesting)
                    {
                        istesting = true;
                        testme.setBackgroundColor(0xFFFF0000);
                        testme.setText("   NAVI_STOP   ");
                        StartTime = System.currentTimeMillis();

                        init_value();

                        arrow_direction.setImageResource(R.drawable.go_straight);  // 다음 state로 넘어갔을때는 직진하는것을 전제로 한다.

                        return_iter++;
                        global_test_state=0;

                        mTimer.cancel();
                        mTimer = new Timer();
                        mTimer.schedule(new OnesecTimer(), 0 , 1000);
                        test_timer = new Timer();
                        test_timer.schedule(new detector(), 0 , 200);
                    }

                    else
                    {
                        istesting = false;
                        testme.setBackgroundResource(android.R.drawable.btn_default);
                        testme.setText("   NAVI_START   ");


                        try {
                            buftotxt("LIGHT","RETURN");
                            buftotxt("ACC","RETURN");
                            buftotxt("GYRO","RETURN");
                            buftotxt("LCS","NORMAL");

                        } catch (IOException e) {
                            e.printStackTrace();
                        }

                        light_current_array.clear();
                        acc_current_array.clear();
                        gyro_current_array.clear();

                        second = 0;

                        mTimer.cancel();
                        mTimer = new Timer();
                        mTimer.schedule(new OnesecTimer(), 0 , 1000);
                        test_timer.cancel();

                        arrow_direction.setImageResource(R.drawable.navigation);

                    }


                }


            });

            btnclear.setOnClickListener(new Button.OnClickListener() {


                public void onClick(View v){

                    light_current_array.clear();
                    acc_current_array.clear();
                    gyro_current_array.clear();
                    normal_trace_array.clear();

                    second = 0;
                    mTimer.cancel();
                    mTimer = new Timer();
                    mTimer.schedule(new OnesecTimer(), 0 , 1000);

                }

            });

            memo_button.setOnClickListener(new Button.OnClickListener() {


                public void onClick(View v){

                    memo_button_pressed=1;
                }

            });
        }

        class OnesecTimer extends TimerTask {
            @Override
            public void run() {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        Time.setText("TIME : " + String.valueOf(Math.round(second))+" second");
                        second++;
                    }
                });

            }
        }

        class detector extends TimerTask {
            @Override
            public void run() {
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {

                        if (istesting) {

                        }

                        // new function

                    }
                });

            }
        }

        @Override
        protected void onStop() {

            super.onStop();
            if (sensorManager != null) {
                sensorManager.unregisterListener(accLis);
                sensorManager.unregisterListener(gyroLis);
                sensorManager.unregisterListener(lightLis);
            }
        }

        private String getTimeSec(){
            mNow = System.currentTimeMillis();
            mDate = new Date(mNow);
            return mFormatSec.format(mDate);
        }

        private void buftotxt(String type, String mode) throws IOException {

            if(type=="LIGHT" || type == "ACC" || type == "GYRO" || type =="DIR" || type =="NORMAL_TRACE") {
                dirPath = Environment.getExternalStorageDirectory().getAbsolutePath() + "/" + "TRACE_LOG" + "/" + Time_Sec + "/";
                File file = new File(dirPath);
                if (!file.exists())
                    file.mkdirs();

                BufferedWriter bfw;

                if(mode == "RETURN")
                    bfw = new BufferedWriter(new FileWriter(dirPath +"R_"+return_iter+"_" +  type + ".csv", true));
                else
                    bfw = new BufferedWriter(new FileWriter(dirPath +  type + ".csv", true));

                if(type=="LIGHT") {
                    Iterator<light_value> iter = light_current_array.iterator();
                    light_value value;

                    while (iter.hasNext()) {
                        value = iter.next();
                        bfw.write(value.time + "," + value.intensity + "," +value.turn_ground_truth);
                        bfw.newLine();
                    }
                }
                else if(type=="ACC") {
                    Iterator<acc_value> iter = acc_current_array.iterator();
                    acc_value value;

                    while (iter.hasNext()) {
                        value = iter.next();
                        bfw.write(value.time + "," + value.x + "," + value.y + "," + value.z+","+value.step + "," + value.light + "," + value.magnitude);
                        bfw.newLine();
                    }
                }
                else if(type=="GYRO"){
                    Iterator<gyro_value> iter = gyro_current_array.iterator();
                    gyro_value value;

                    while (iter.hasNext()) {
                        value = iter.next();
                        bfw.write(value.time + "," + value.i_z);
                        bfw.write("," + value.z + "," + value.turn_flag + ","+value.turn_degree);
                        bfw.newLine();
                    }
                }

                else if(type=="NORMAL_TRACE") {
                    Iterator<normal_trace> iter = normal_trace_array.iterator();
                    normal_trace value;

                    while (iter.hasNext()) {
                        value = iter.next();
                        bfw.write(value.time + "," + value.lux + "," + value.lux_peak + "," + value.step+"," + value.turn_flag + ","+value.turn_degree + "," + value.stair_flag);
                        bfw.newLine();
                    }
                }



                bfw.flush();
                bfw.close();
            }
        }

        public void init_value(){

            filtered_acc=0;
            acc_tendency=0;
            stepCount = 0;
            previous_step_time=0;
            previous_step_count=0;
            light_peak_count=0;
            light_tendency=0;
            gyro_tendency=0;
            pre_gyro_value=0;
            second = 0;
            cur_test_peak_count=0;
            current_turn_flag=0;
            aggRotation=0;
            filteredRotation=0;
        }

        public void gyro_array_degree_finder(){

            int j = 0 ;

            for (int i = 0; i < gyro_current_array.size(); i ++) {
                // i is the index
                if(gyro_current_array.get(i).turn_flag==1)
                {
                    // 턴하는 센터 index를 찾았음
                    int center_left = i-1;
                    int center_right = i+1;

                    while(Math.abs(gyro_current_array.get(center_left).i_z)>ANGLE_DETECTION_ACC_THRESHOLD)
                    {
                        center_left--;
                    }

                    while(Math.abs(gyro_current_array.get(center_right).i_z)>ANGLE_DETECTION_ACC_THRESHOLD)
                    {
                        center_right++;
                    }

                    // 위까지 하면 양끝 ANGLE_DETECTION_ACC_THRESHOLD에 최대한 가까운 index값을 찾을 수 있다. 현재 값은 0.2

                    double found_degree = (gyro_current_array.get(center_right).z-gyro_current_array.get(center_left).z);
                    gyro_current_array.get(i).turn_degree=found_degree;

                    // 다 찾았으면 normal_array에 넣어줘야한다.
                    // 순서만 잘 맞추면 문제없음.

                    for (; j<normal_trace_array.size(); j++)
                    {
                        if(Math.abs(normal_trace_array.get(j).turn_flag) == 1 )
                        {
                            normal_trace_array.get(j).turn_degree=found_degree;
                        }

                    }

                }

            }

        }

        @Override
        public void onDestroy() {
            super.onDestroy();
        }


        protected void onStart() {
            super.onStart();

            if (accSensor != null)
                sensorManager.registerListener(accLis, accSensor, SensorManager.SENSOR_DELAY_FASTEST);
            if (gyroSensor != null)
                sensorManager.registerListener(gyroLis, gyroSensor, SensorManager.SENSOR_DELAY_FASTEST);
            if (lightSensor != null)
                sensorManager.registerListener(lightLis, lightSensor, SensorManager.SENSOR_DELAY_FASTEST);
        }

        private class lightmeterlistener implements SensorEventListener {

            @Override
            public void onSensorChanged(SensorEvent event) {

                filtered_light = filtered_light * 0.75 + 0.25* event.values[0];

                double light_cur_tendency = Math.signum(filtered_light - pre_light_value);

                if(isrunning) {

                    light_value light_put = new light_value();
                    light_put.intensity = filtered_light;

                    t_light.setText("LUX  : " + Math.round(filtered_light));  // 현재 찍히는 LUX의 RAW 데이터를 표시한다.

                    double light_del_time = System.currentTimeMillis()-previous_light_peak_time;

                    if(Math.signum(light_cur_tendency*light_tendency)<0 && pre_light_value > 80 && (light_del_time>3000 || previous_light_peak_time==0) && light_tendency > 0)
                    {
                        light_peak_count++;
                        previous_light_peak_time = System.currentTimeMillis();
                    }


                    ///  FOR NORMAL TRACE

                    normal_trace normal_put = new normal_trace();
                    normal_put.lux = filtered_light;
                    normal_put.lux_peak= light_peak_count;
                    normal_put.time = System.currentTimeMillis() - StartTime;
                    normal_put.turn_flag=0;
                    normal_put.step=previous_step_count;
                    normal_trace_array.add(normal_put);

                    ///  END FOR NORMAL TRACE


                    light_put.peak = light_peak_count;
                    light_put.time = System.currentTimeMillis() - StartTime;

                    if(memo_button_pressed==1)
                    {
                        light_put.turn_ground_truth = 1;
                        memo_button_pressed = 0;
                    }
                    else
                    {
                        light_put.turn_ground_truth = 0;
                    }
                    light_current_array.add(light_put);
                }

                else if(istesting) {

                    light_value light_put = new light_value();
                    light_put.intensity = filtered_light;

                    t_light.setText("LUX  : " + Math.round(filtered_light));

                    double light_del_time = System.currentTimeMillis()-previous_light_peak_time;

                    if(Math.signum(light_cur_tendency*light_tendency)<0 && pre_light_value > 80 && (light_del_time>3000 || previous_light_peak_time==0) && light_tendency > 0)
                    {
                        light_peak_count++;
                        previous_light_peak_time = System.currentTimeMillis();
                    }
                    light_put.peak = light_peak_count;
                    light_put.time = System.currentTimeMillis() - StartTime;
                    light_current_array.add(light_put);

                }

                pre_light_value = filtered_light;
                light_tendency=light_cur_tendency;
            }

            @Override
            public void onAccuracyChanged(Sensor sensor, int i) {

            }
        };

        private class accelerometerListener implements SensorEventListener {

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
            }

            public void onSensorChanged(SensorEvent event) {
                Sensor accSensor = event.sensor;
                if (accSensor.getType() == Sensor.TYPE_LINEAR_ACCELERATION) {

                    if(isrunning || istesting) {
                        acc_value acc_put = new acc_value();
                        if(event.values[0] < 0.1)
                        acc_put.x = event.values[0];
                        else
                            acc_put.x = 0;
                        if(event.values[1] < 0.1)
                            acc_put.y = event.values[1];
                        else
                            acc_put.y = 0;
                        if(event.values[2] < 0.1)
                            acc_put.z = event.values[2];
                        else
                            acc_put.z = 0;

                        double acc_mag = Math.sqrt(acc_put.x*acc_put.x+acc_put.y*acc_put.y+acc_put.z*acc_put.z);

                        filtered_acc = filtered_acc * 0.9 + 0.1* acc_mag;
                        double acc_cur_tendency = Math.signum(filtered_acc - pre_acc_value);
                        acc_put.magnitude=filtered_acc;

                        double step_del_time = System.currentTimeMillis()-previous_step_time;

                        if(Math.signum(acc_cur_tendency*acc_tendency)<0 && pre_acc_value > 1.5 && step_del_time>300)
                        {
                            stepCount++;
                            previous_step_time=System.currentTimeMillis();

                            ///  FOR NORMAL TRACE   // STEP이 올라갈때마다.

                            normal_trace normal_put = new normal_trace();
                            normal_put.lux = filtered_light;
                            normal_put.lux_peak= light_peak_count;
                            normal_put.time = System.currentTimeMillis() - StartTime;
                            normal_put.turn_flag=0;
                            normal_put.step=stepCount;

                            previous_step_count=stepCount;

                            normal_trace_array.add(normal_put);

                            ///  END FOR NORMAL TRACE

                        }

                        acc_put.step = stepCount;

                        acc_tendency = acc_cur_tendency;

                        acc_put.light= pre_light_value;

                        pre_acc_value = filtered_acc;

                        acc_put.time = System.currentTimeMillis() - StartTime;

                        acc_current_array.add(acc_put);

                    }
                }
            }
        }

        //GYRO
        private class gyroscopeListener implements SensorEventListener {

            private double curTime, preTime, dt;
            private double rotationThreshold = 0.2;  // reduce noise for walking scenario
            private double aggRotationThreshold = 5; /** degree **/

            @Override
            public void onAccuracyChanged(Sensor sensor, int accuracy) {
            }

            @Override
            public void onSensorChanged(SensorEvent event) {
                float alpha = (float) 0.9;
                //long time = (long) (curTime - sysTime);
                curTime = System.currentTimeMillis();
                if (preTime == 0.0) {
                    preTime = curTime;
                }
                dt = curTime - preTime;

                if(Math.abs(event.values[2]) < rotationThreshold)
                    rotation = alpha * rotation + 0;
                else
                    rotation = alpha * rotation + (1 - alpha) * event.values[2];

                aggRotation = (float) (aggRotation + (rotation * dt * MS2S));


                preTime = curTime;

                        //if ((Math.abs(aggRotation[i] * RAD2DGR) > aggRotationThreshold)) {
                if ((Math.abs(aggRotation * RAD2DGR) > aggRotationThreshold))
                {
                    filteredRotation = filteredRotation + aggRotation;
                    if (filteredRotation > 2*Math.PI)
                        filteredRotation = (float) (filteredRotation - 2*Math.PI);
                    if (filteredRotation < -2*Math.PI)
                        filteredRotation = (float) (filteredRotation + 2*Math.PI);
                    aggRotation = 0;
                }

                if(isrunning || istesting) {

                    double gyro_cur_tendency = Math.signum(rotation - pre_gyro_value);

                    gyro_value gyro_put = new gyro_value();

                    gyro_put.z = filteredRotation * (float) RAD2DGR;

                    gyro_put.i_z = rotation;

                    if(Math.signum(gyro_cur_tendency*gyro_tendency)<0 && Math.abs(pre_gyro_value) > 2) {      // 급격하게 턴 했을때 그 턴을 감지한다.

                        if(pre_gyro_value>0)
                        {
                            gyro_put.turn_flag=1;

                            if(istesting)
                            current_turn_flag=1;

                            ///  FOR NORMAL TRACE   // GYRO에서 디텍션 될 떄 마다

                            normal_trace normal_put = new normal_trace();
                            normal_put.lux = filtered_light;
                            normal_put.lux_peak= light_peak_count;
                            normal_put.time = System.currentTimeMillis() - StartTime;
                            normal_put.turn_flag=1;
                            normal_put.step=previous_step_count;

                            normal_trace_array.add(normal_put);

                            // FOR END NORMAL TRACE
                        }
                        else
                        {
                            gyro_put.turn_flag= 1;

                            if(istesting)
                            current_turn_flag = -1;

                                ///  FOR NORMAL TRACE   // GYRO에서 디텍션 될 떄 마다

                            normal_trace normal_put = new normal_trace();
                            normal_put.lux = filtered_light;
                            normal_put.lux_peak= light_peak_count;
                            normal_put.time = System.currentTimeMillis() - StartTime;
                            normal_put.turn_flag=-1;
                            normal_put.step=previous_step_count;

                            normal_trace_array.add(normal_put);

                             // FOR END NORMAL TRACE

                        }

                    }

                    pre_gyro_value = rotation;
                    gyro_tendency = gyro_cur_tendency;

                    t_gyro_x.setText("Degree : " + Math.round(gyro_put.z) +"°");
                    //display

                    gyro_put.time =System.currentTimeMillis() - StartTime;
                    gyro_current_array.add(gyro_put);
                }

            }
        }

        public class gyro_value {
            long time;
            float z;
            double i_z;
            int turn_flag=0;
            double turn_degree;
        }

        public class acc_value {
            long time;
            double x;
            double y;
            double z;
            int step;
            double magnitude;
            double light;
        }

        public class normal_trace{
            long time;
            double lux;
            int lux_peak;
            int step;
            int turn_flag;
            int stair_flag;
            double turn_degree;

        }

        public class light_value{
            long time;
            double intensity;
            int peak;
            int turn_ground_truth;
        }
    }

