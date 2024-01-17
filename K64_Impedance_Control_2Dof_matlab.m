function output_data = K64_Impedance_Control_2Dof_matlab()
    figure(1);  clf;       
    a1 = subplot(421);
    h1 = plot([0],[0]);
    h1.XData = []; h1.YData = [];
    ylabel('Angle 1 (rad)');
    
    a2 = subplot(423);
    h2 = plot([0],[0]);
    h2.XData = []; h2.YData = [];
    ylabel('Velocity 1 (rad/s)');
    
    a3 = subplot(425);
    h3 = plot([0],[0]);
    h3.XData = []; h3.YData = [];
    ylabel('Current 1 (A)');
    hold on;
    subplot(425);
    h4 = plot([0],[0],'r');
    h4.XData = []; h4.YData = [];
    hold off;
    
    a4 = subplot(427);
    h5 = plot([0],[0]);
    h5.XData = []; h5.YData = [];
    ylabel('Duty Ratio 1 (%)');
    

    a5 = subplot(422);
    h12 = plot([0],[0]);
    h12.XData = []; h12.YData = [];
    ylabel('Angle 2 (rad)');
    
    a6 = subplot(424);
    h22 = plot([0],[0]);
    h22.XData = []; h22.YData = [];
    ylabel('Velocity (rad/s)');
    
    a7 = subplot(426);
    h32 = plot([0],[0]);
    h32.XData = []; h32.YData = [];
    ylabel('Current 2 (A)');
    hold on;
    subplot(426);
    h42 = plot([0],[0],'r');
    h42.XData = []; h42.YData = [];
    hold off;
    a8 = subplot(428);
    h52 = plot([0],[0]);
    h52.XData = []; h52.YData = [];
    ylabel('Duty Ratio 2 (%)');
    
    
    figure(3);  clf;   
    subplot(321)
    g1 = plot([0],[0]);
    g1.XData = [];
    g1.YData = [];
    ylabel('X Foot Position (m)');
    hold on;
    g12 = plot([0],[0],'r');
    g12.XData = [];
    g12.YData = [];
    hold off;
    
    subplot(322)
    g2 = plot([0],[0]);
    g2.XData = [];
    g2.YData = [];
    ylabel('Y Foot Position (m)');
    hold on;
    g22 = plot([0],[0],'r');
    g22.XData = [];
    g22.YData = [];
    hold off;
    
    subplot(323)
    g3 = plot([0],[0]);
    g3.XData = [];
    g3.YData = [];
    ylabel('X Foot Velocity (m/s)');
    hold on;
    g32 = plot([0],[0],'r');
    g32.XData = [];
    g32.YData = [];
    hold off;
    
    subplot(324)
    g4 = plot([0],[0]);
    g4.XData = [];
    g4.YData = [];
    ylabel('Y Foot Velocity (m/s)');
    hold on;
    g42 = plot([0],[0],'r');
    g42.XData = [];
    g42.YData = [];
    hold off;
    
    subplot(325)
    g5 = plot([0],[0]);
    g5.XData = [];
    g5.YData = [];
    ylabel('X Foot Force (m)');
    
    subplot(326)
    g6 = plot([0],[0]);
    g6.XData = [];
    g6.YData = [];
    ylabel('Y Foot Force (m)');
    
    
    h_OB = 0;
    h_AC = 0;
    h_BD = 0;
    h_CE = 0;
    
    %% Definte fixed paramters
    m1 =.03;
    m2 =.01; 
    m3 = .01;
    m4 = .02;
    I1 = .0005;
    I2 = .0005;
    I3 = .0005;
    I4 = .0005;
    l_OA=.011; 
    l_OB=.042; 
    l_AC=.096; 
    l_DE=.090;
    
    l_O_m1=1/2 * l_OB; 
    l_B_m2=1/2 * l_AC; 
    l_A_m3=1/2 * l_AC; 
    l_C_m4=1/2 * (l_DE+ l_OB-l_OA);
    g = 10*0;

    p   = [m1 m2 m3 m4 I1 I2 I3 I4 l_O_m1 l_B_m2 l_A_m3 l_C_m4 l_OA l_OB l_AC l_DE g]';
    
    % This function will get called any time there is new data from
    % the FRDM board. Data comes in blocks, rather than one at a time.
    function my_callback(new_data)
        t = new_data(:,1);   % time
        pos1 = new_data(:,2); % position
        vel1 = new_data(:,3); % velocity
        cur1 = new_data(:,4); % current
        dcur1 = new_data(:,5); % current
        duty1 = new_data(:,6); % current
        
        pos2 = new_data(:,7); % position
        vel2 = new_data(:,8); % velocity
        cur2 = new_data(:,9); % current
        dcur2 = new_data(:,10); % current
        duty2 = new_data(:,11); % current
        
        x = new_data(:,12); % current
        y = new_data(:,13); % current
        
        dx = new_data(:,14); % current
        dy = new_data(:,15); % current
        
        fx = new_data(:,16); % current
        fy = new_data(:,17); % current

        xd = new_data(:,18); % current
        yd = new_data(:,19); % current
        dxd = new_data(:,20); % current
        dyd = new_data(:,21); % current
        
%         
        
        N = length(pos1);
        
        h1.XData(end+1:end+N) = t;   
        h1.YData(end+1:end+N) = pos1;
        h2.XData(end+1:end+N) = t;   
        h2.YData(end+1:end+N) = vel1;
        h3.XData(end+1:end+N) = t;   
        h3.YData(end+1:end+N) = cur1;
        h4.XData(end+1:end+N) = t;   
        h4.YData(end+1:end+N) = dcur1;
        h5.XData(end+1:end+N) = t;   
        h5.YData(end+1:end+N) = duty1;
        
        h12.XData(end+1:end+N) = t;   
        h12.YData(end+1:end+N) = pos2;
        h22.XData(end+1:end+N) = t;   
        h22.YData(end+1:end+N) = vel2;
        h32.XData(end+1:end+N) = t;   
        h32.YData(end+1:end+N) = cur2;
        h42.XData(end+1:end+N) = t;  
        h42.YData(end+1:end+N) = dcur2;
        h52.XData(end+1:end+N) = t;   
        h52.YData(end+1:end+N) = duty2;
        
        g1.XData(end+1:end+N) = t;   
        g1.YData(end+1:end+N) = x;
        g12.XData(end+1:end+N) = t;   
        g12.YData(end+1:end+N) = xd;
        g2.XData(end+1:end+N) = t;   
        g2.YData(end+1:end+N) = y;
        g22.XData(end+1:end+N) = t;   
        g22.YData(end+1:end+N) = yd;
        g3.XData(end+1:end+N) = t;  
        g3.YData(end+1:end+N) = dx;
        g32.XData(end+1:end+N) = t;  
        g32.YData(end+1:end+N) = dxd;
        g4.XData(end+1:end+N) = t;   
        g4.YData(end+1:end+N) = dy;
        g42.XData(end+1:end+N) = t;   
        g42.YData(end+1:end+N) = dyd;
        g5.XData(end+1:end+N) = t;  
        g5.YData(end+1:end+N) = fx;
        g6.XData(end+1:end+N) = t;   
        g6.YData(end+1:end+N) = fy;
        
        
        z = [pos1(end) pos2(end) vel1(end) vel2(end)]';
        keypoints = keypoints_leg(z,p);
        
        rA = keypoints(:,1); 
        rB = keypoints(:,2);
        rC = keypoints(:,3);
        rD = keypoints(:,4);
        rE = keypoints(:,5);

        set(h_OB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
        set(h_AC,'XData',[rA(1) rC(1)],'YData',[rA(2) rC(2)]);
        set(h_BD,'XData',[rB(1) rD(1)],'YData',[rB(2) rD(2)]);
        set(h_CE,'XData',[rC(1) rE(1)],'YData',[rC(2) rE(2)]);
    end
    
    %%
    frdm_ip  = '192.168.1.100';     % FRDM board ip
    frdm_port= 11223;               % FRDM board port  
    params.callback = @my_callback; % callback function
              % end of experiment timeout
    
    
    %% Parameters for tuning
    current_control_period_us   = 200;  % Current control period in micro seconds
    impedance_control_period_us = 2000; % Impedance control period in microseconds seconds
    exp_period                  = 14;   % Experiment time in seconds 

    Rm                        = 1.9; % Terminal resistance (Ohms)
    Kb                        = .1375; % Back EMF Constant (V / (rad/s))
    Kv                        = 5e-4;% Friction coefficienct (Nm / (rad/s))
    
    supply_voltage           = 12;  % Power Supply Voltage (V)

    angle1_init              = -1.3910; % Initial angle for q1 (rad)
    angle2_init              = 0.4414; % Initial angle for q2 (rad)

    Kp                      = 2;  % Proportional current gain (V/A)
    Ki                      = 0.1; % Integral gain of current controler

    // *** Cartesian Gains *** //
    K_xx                    = 30; % Stiffness
    K_yy                    = 40; % Stiffness
    K_xy                    = -0.0; % Stiffness

    D_xx                     = 0.5; % Damping
    D_yy                     = 0.5; % Damping
    D_xy                     = 0.000; % Damping
    // *** End ***///
    
    xDesFoot                 = 0;     % Desired foot position x (m)
    yDesFoot                 = -0.13; % Desired foot position y (m)
    A                        = 0.06;  % Magnitude of oscillation (m)
    omega                    = 1;     % Angular velocity of oscillation (RPS, Revolutions Per Second)

    duty_max                 = 1;  % Maximum PWM duty (safety limit)
    
    %% Sepectify inputs
    input = [current_control_period_us impedance_control_period_us exp_period];
    input = [input Rm Kb Kv supply_voltage angle1_init angle2_init];
    input = [input Kp Ki K_xx K_yy K_xy D_xx D_yy D_xy xDesFoot yDesFoot A omega];
    input = [input duty_max];
    params.timeout  = exp_period;  
    
    
    %% Plot and go
    global X Y V
    figure(2)
    clf
    hold on
    axis equal
    axis([-.25 .25 -.25 .1]);
    [X Y] = meshgrid(linspace(-.25,.25,50),linspace(-.25, .1,50));
    eX = X - xDesFoot;
    eY = Y - yDesFoot;
    V = 1/2 * K_xx * eX.*eX + 1/2 * K_yy * eY.*eY + K_xy * eX.*eY ;
    contour(X,Y,V,15,'LineWidth',1.5);
   
    h_OB = plot([0],[0],'LineWidth',3);
    h_AC = plot([0],[0],'LineWidth',3);
    h_BD = plot([0],[0],'LineWidth',3);
    h_CE = plot([0],[0],'LineWidth',3);
    
    z = [pi/4 -pi/1.7 0 0 ]';
    keypoints = keypoints_leg(z,p);

    rA = keypoints(:,1); 
    rB = keypoints(:,2);
    rC = keypoints(:,3);
    rD = keypoints(:,4);
    rE = keypoints(:,5);

    set(h_OB,'XData',[0 rB(1)],'YData',[0 rB(2)]);
    set(h_AC,'XData',[rA(1) rC(1)],'YData',[rA(2) rC(2)]);
    set(h_BD,'XData',[rB(1) rD(1)],'YData',[rB(2) rD(2)]);
    set(h_CE,'XData',[rC(1) rE(1)],'YData',[rC(2) rE(2)]);
    xlabel('X Position (m)');
    ylabel('Y Position (m)');
    colormap()
        
    
    output_size = 21;    % number of outputs expected
    output_data = RunExperiment(frdm_ip,frdm_port,input,output_size,params);
    %linkaxes([a1 a2 a3 a4],'x')
end