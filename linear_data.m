% Model         :   PMSM Field Oriented Control
% Description   :   Set Parameters for PMSM Field Oriented Control
% File name     :   mcb_pmsm_foc_qep_f28379d_data.m

% Copyright 2020-2022 The MathWorks, Inc.
%clear,clc
%% Set PWM Switching frequency
PWM_frequency 	= 20e3;             %Hz     // converter s/w freq
T_pwm           = 1/PWM_frequency;  %s      // PWM switching time period

%% Set Sample Times
Ts          	= T_pwm;            %sec    // Sample time for control system
Ts_simulink     = T_pwm/2;          %sec    // Simulation time step for model simulation
Ts_motor        = T_pwm/2;          %Sec    // Simulation time step for pmsm
Ts_inverter     = T_pwm/2;          %sec    // Simulation time step for inverter
Ts_speed        = 1*Ts;            %Sec    // Sample time for speed controller

%% Set data type for controller & code-gen
% dataType = fixdt(1,32,17);        % Fixed point code-generation
dataType = 'single';                % Floating point code-generation

%% System Parameters 
%Motor1 parameters 选择对应的电机
pmsm.model    = 'E-RAM10-S';        %            // MISUMI 直线电机模组10系列
pmsm.sn       = 'linear-1';         %            // 等效旋转模型，1转=20mm
pmsm.p        = 1;                  %            // 极对数，电周期2*tau=20mm
pmsm.Rs       = 3.6;                %Ohm        // 相间7.2转星形单相值
pmsm.Ld       = 13.85e-3;           %H          // 相间27.7mH单相值
pmsm.Lq       = 13.85e-3;           %H          // 表贴式，Ld=Lq
pmsm.J        = 2.2291e-05;         %Kg-m2      // M*r^2, M=2.2kg, r=p*tau/pi
pmsm.B        = 4.3734e-07;         %Kg-m2/s    // 阻力0.065N@1.5m/s折算黏滞
pmsm.Ke       = 36.2;               %Vpk_LL/(m/s) // 手册反电动势
pmsm.I_rated  = 3.5355;             %A      	// 2.5Arms转相峰值
pmsm.N_base   = 1500;               %mm/s       // 手册额定1.5m/s
pmsm.N_max    = 2000;               %mm/s       // 手册最高2.0m/s
pmsm.PositionOffset = 0;	        %PU position
pmsm.QEPSlits = 5000;               %           // 四倍频,1um分辨率, 20mm/转=20000

%% 兼容模型里的直线电机负载/速度换算引用（pmlsm / speedRPM2mmps）
pmlsm.F_fric_rated = 0.0647;                  %N   μ=0.003×2.2kg×g
pmlsm.tau_elec     = 20e-3;                   %m   电周期长度（= 2*tau 极距）
pmlsm.tau          = pmlsm.tau_elec/2;        %m   极距 10mm
pmlsm.rpm2mps      = pmsm.p * pmlsm.tau / 30; %    v[m/s] = rpm * rpm2mps
pmlsm.mps2rpm      = 1/pmlsm.rpm2mps;
pmlsm.mmps_per_rpm = 1000 * pmsm.p * pmlsm.tau / 30;  % 0.3333 mm/s per rpm
speedRPM2mmps      = pmlsm.mmps_per_rpm;      % 模型增益 speed_scale_mmps 用

pmsm.FluxPM   = pmsm.Ke/sqrt(3)*pmlsm.tau_elec/(2*pi); %PM flux由反电动势反算
pmsm.Kt       = 1.5*pmsm.p*pmsm.FluxPM;                %Nm/A 推力常数折算
pmsm.T_rated  = mcbPMSMRatedTorque(pmsm);   %Get T_rated from I_rated

% Motor2 parameters
pmsm2 = pmsm;

%% Target & Inverter Parameters
target = mcb_SetProcessorDetails('F28379D',PWM_frequency);
%target.comport = '<Select a port...>';
target.comport = 'COM8';
inverter.model         = 'BoostXL-DRV8323'; 	% 		// Manufacturer Model Number
inverter.sn            = 'INV_XXXX';         	% 		// Manufacturer Serial Number
inverter.V_dc          = 24;       				%V      // DC Link Voltage of the Inverter
inverter.I_trip        = 18.5;       			%Amps   // Max current for trip
inverter.Rds_on        = 0.004;     			%Ohms   // Rds ON for BoostXL-DRV8305
inverter.Rshunt        = 0.007;    				%Ohms   // Rshunt for BoostXL-DRV8305
inverter.R_board         = inverter.Rds_on + inverter.Rshunt/3;  %Ohms

inverter.CtSensAOffset = 1988;        			%Counts // ADC Offset for phase-A  
inverter.CtSensBOffset = 1990;        			%Counts // ADC Offset for phase-B
inverter.CtSensCOffset = 1979;        			%Counts // ADC Offset for phase-C

inverter.CtSensOffsetMax = 3000;                % Maximum permitted ADC counts for current sense offset
inverter.CtSensOffsetMin = 1500;                % Minimum permitted ADC counts for current sense offset
inverter.EnableLogic   = 1;    					% 		// Active high for DRV8305 enable pin (EN_GATE)
inverter.invertingAmp  = 1;   					% 		// Currents entering motor phases are read as positive values in this hardware
inverter.ISenseVref    = 3.3;					%V 		// Voltage ref of inverter current sense circuit
inverter.CSAGain = 20;                          %        //逆变器CSA增益 200W电机增益为20 600W为10
inverter.ISenseVoltPerAmp = inverter.Rshunt*inverter.CSAGain; %V/Amps // Current sense voltage output per 1 A current (Rshunt * iSense op-amp gain)
inverter.ISenseMax     = inverter.ISenseVref/(2*inverter.ISenseVoltPerAmp); %Amps // Maximum Peak-Neutral current that can be measured by inverter current sense
inverter.ISenseMax = inverter.ISenseMax * target.ADC_Vref / inverter.ISenseVref;%3V ADC采样对应的最大电流更新
inverter.ADCOffsetCalibEnable = 0;  % Enable: 1, Disable:0 % Enable automatic calibration of ADC offset for current measurement

%逆变器2参数
inverter2.model         = 'BoostXL-DRV8323'; 	    % 		// Manufacturer Model Number
inverter2.sn            = 'INV_XXXX';         	    % 		// Manufacturer Serial Number
inverter2.V_dc          = inverter.V_dc;       		%V      // DC Link Voltage of the Inverter
inverter2.I_trip        = 18.5;       				    %Amps   // Max current for trip
inverter2.Rds_on        = 0.004;     			    %Ohms   // Rds ON for BoostXL-DRV8305   mosfet导通后电阻
inverter2.Rshunt        = 0.007;    				%Ohms   // Rshunt for BoostXL-DRV8305   采样电阻
inverter2.R_board         = inverter2.Rds_on + inverter2.Rshunt/3;  %Ohms

inverter2.CtSensAOffset = 2026;        			    %MY Counts // ADC Offset for phase-A
inverter2.CtSensBOffset = 1976;        			    %Counts // ADC Offset for phase-B
inverter2.CtSensCOffset = 1985;        			    %Counts // ADC Offset for phase-C

inverter2.CtSensOffsetMax = 3000;                   %       // Maximum permitted ADC counts for current sense offset
inverter2.CtSensOffsetMin = 1500;                   %       // Minimum permitted ADC counts for current sense offset
inverter2.EnableLogic   = 1;    				    % 		// Active high for DRV8323 enable pin (EN_GATE)
inverter2.invertingAmp  = 1;   					    % 		// Currents entering motor phases are read as positive values in this hardware
inverter2.ISenseVref    = 3.3;					    %V 		// Voltage ref of inverter current sense circuit
inverter2.CSAGain = inverter.CSAGain;               %        //电流放大增益   逆变器CSA增益 200W电机增益为20 600W为10
inverter2.ISenseVoltPerAmp = inverter2.Rshunt*inverter2.CSAGain; 			        %V/Amps // Current sense voltage output per 1 A current (Rshunt * iSense op-amp gain)
inverter2.ISenseMax     = inverter2.ISenseVref/(2*inverter2.ISenseVoltPerAmp);      %Amps   // Maximum Peak-Neutral current that can be measured by inverter current sense
inverter2.ISenseMax = inverter2.ISenseMax * target.ADC_Vref / inverter2.ISenseVref; %3V ADC采样对应的最大电流更新
inverter2.ADCOffsetCalibEnable = 0;  % Enable: 1, Disable:0 % Enable automatic calibration of ADC offset for current measurement

%% Derive Characteristics
pmsm.N_base = mcb_getBaseSpeed(pmsm,inverter)*pmlsm.mmps_per_rpm; %mm/s // 母线限制下的基速

% mcb_getCharacteristics(pmsm,inverter);

%% PU System details // Set base values for pu conversion

PU_System = mcb_SetPUSystem(pmsm,inverter);

%% Controller design // Get ballpark values!
PI_params = mcb_SetControllerParameters(pmsm,inverter,PU_System,T_pwm,Ts,Ts_speed);

%Updating delays for simulation
PI_params.delay_Currents    = int32(Ts/Ts_simulink);
PI_params.delay_Position    = int32(Ts/Ts_simulink);
PI_params.delay_Speed       = int32(Ts_speed/Ts_simulink);
PI_params.delay_Speed1      = (PI_params.delay_IIR + 0.5*Ts)/Ts_speed;

% mcb_getControlAnalysis(pmsm,inverter,PU_System,PI_params,Ts,Ts_speed); 


%% PMSM Current Controller %电流环整定
innerPI.Filter.w0 = 2*pi*2000;      %电流环低通滤波器频率：2kHZ

% D axis PI design %Controller Structure:K(s) = Kp + Ki/s
%innerPI.iD.f0 = 1.5e+03;         %D电流环带宽频率
%innerPI.iD.f0 = 7.5e+03;
innerPI.iD.f0 = 800;
innerPI.iD.w0 = 2*pi*innerPI.iD.f0; %D电流环带宽角频率

innerPI.iD.Continuous.Kp = innerPI.iD.w0*pmsm.Ld;
innerPI.iD.Continuous.Ki = innerPI.iD.w0*pmsm.Rs;

innerPI.iD.Discrete.Kp = innerPI.iD.Continuous.Kp;
innerPI.iD.Discrete.Ki = innerPI.iD.Continuous.Ki;
innerPI.iD.Discrete.KiTctr = innerPI.iD.Discrete.Ki*Ts;

% innerPI.iD.Discrete.Kp = 26.0;
% innerPI.iD.Discrete.Ki = 6783;
% innerPI.iD.Discrete.KiTctr = 0.339;

% Q axis PI design %Controller Structure:K(s) = Kp + Ki/s
%innerPI.iQ.f0 = 1.5e+03;
%innerPI.iQ.f0 = 7.5e+03;
innerPI.iQ.f0 = 800;
innerPI.iQ.w0 = 2*pi*innerPI.iQ.f0;

innerPI.iQ.Continuous.Kp = innerPI.iQ.w0*pmsm.Lq;
innerPI.iQ.Continuous.Ki = innerPI.iQ.w0*pmsm.Rs;

innerPI.iQ.Discrete.Kp = innerPI.iQ.Continuous.Kp;
innerPI.iQ.Discrete.Ki = innerPI.iQ.Continuous.Ki;
innerPI.iQ.Discrete.KiTctr = innerPI.iQ.Discrete.Ki*Ts;

% innerPI.iQ.Discrete.Kp = 26.0;
% innerPI.iQ.Discrete.Ki = 6783;
% innerPI.iQ.Discrete.KiTctr = 0.339;

%% Speed Controller 速度环整定
%outerPI.Spd.Continuous.Kp = 100.0;
%outerPI.Spd.Continuous.Ki = 100.0;
outerPI.Spd.Continuous.Kp = 25;
outerPI.Spd.Continuous.Ki = 750;
% %%原跑实验参考值（高频带宽易受10ms测速窗滞后影响）
% outerPI.Spd.Continuous.Kp = 70.6234;
% outerPI.Spd.Continuous.Ki = 60.0;

outerPI.Spd.Continuous.Kzc = outerPI.Spd.Continuous.Kp/outerPI.Spd.Continuous.Ki;

outerPI.Spd.Discrete.Kp = outerPI.Spd.Continuous.Kp;
outerPI.Spd.Discrete.Ki = outerPI.Spd.Continuous.Ki;
outerPI.Spd.Discrete.KiTctr = outerPI.Spd.Discrete.Ki*Ts;

%% Displaying model variables
disp(pmsm);disp(pmsm2);
disp(inverter);disp(inverter2);
disp(target);
disp(PU_System);

ADRC_Wo = 30;
ADRC_Wc = 3.2e-3;
ADRC_B0 = 6055;

%%ppc
N_pu_base_rpm = pmsm.N_max/pmlsm.mmps_per_rpm;  %rpm 等效转速基准(N_max为mm/s)
lambda_ppc = 45;                    % 1/s 
alpha_ppc  = exp(-lambda_ppc*Ts_speed);
k_xi       = 55;                    % 1/s
omega_pu_base = 2*pi*N_pu_base_rpm/60;   % rad/s
FluxPM = pmsm.FluxPM;               % 与pmsm一致(Ke已是Vpk_LL/(m/s))
Kt_eff = single(1.5) * pmsm.p * FluxPM; 
a_hat = Kt_eff / (pmsm.J * omega_pu_base);
b_hat      = pmsm.B/pmsm.J;
gamma_reset = 0.9;
s_limit_sat = 0.9;
P_inf_const = 0.004;
inv_a_hat = 1/double(a_hat);        % 由a_hat反算，换电机后不再写死
Kff = 0.98931745;
beta2_sat_scale = 0.2;

w_o = single(200.0);           % 观测器带宽 (rad/s)，可根据反馈噪声大小调整(建议100~400)
beta1 = single(2.0 * w_o);     % ESO 增益 1
beta2 = single(w_o * w_o);     % ESO 增益 2
z2_limit = single(100.0);     % 扰动估计限幅(防止启动/大阶跃时积分饱和发散)


CalModeP = 0;

%% 空间改进型重复控制器 (SMRC) 参数 (参考文献 IEEE TAC 2024 Eq.(13),(52)-(54))
SMRC_Gamma_mm  = single(pmlsm.tau_elec * 1000); % 空间周期 Gamma = 20 mm (1个电周期)
SMRC_tau_mm    = single(0.8);     % 位置域低通滤波常数 tau (mm), 与 Gamma/dx 同单位
SMRC_alpha_mm  = single(0.8);     % 周期对齐因子 alpha (mm), 取 alpha = tau (Eq.21)
SMRC_q0        = single(0.95);    % 空间低通遗忘因子 q0 in (0,1], 保证小增益稳定裕度
SMRC_Krc       = single(1.0);     % SMRC 插入比例系数 (0~1; 设为 0 可关闭 SMRC 对比纯 PI)
SMRC_Substeps  = int32(10);       % 分频步数: Ts_smrc = 10 * Ts_speed = 0.5 ms (2 kHz)
SMRC_dx_min_mm = single(0.005);   % 最低有效位移门限 (mm/步, 对应 ~10 mm/s), 静止/反向冻结
SMRC_xf_limit  = single(0.5);     % 内部记忆状态限幅 (PU)

%% 电流环空间改进型重复控制器 (d/q 轴共用滑窗 SMRC) 参数
% 全电周期 Gamma = 20 mm，同时覆盖 1/2 次采样与相不平衡误差及 6/12 次死区/反电势谐波
SMRC_I_Gamma_mm    = single(pmlsm.tau_elec * 1000); % 空间周期 20 mm (1个电周期)
SMRC_I_tau_mm      = single(0.20);   % 位置域低通滤波常数 tau_i (mm)
SMRC_I_alpha_mm    = single(0.20);   % 周期对齐因子 alpha_i = tau_i (mm)
SMRC_I_q0          = single(0.95);   % 空间遗忘因子 q0 in (0,1]
SMRC_I_Krc         = single(0.85);   % 插入增益 (设为 0 可关闭对比纯电流 PI)
SMRC_I_Substeps    = int32(4);       % 分频步数: Ts_smrc_i = 4*Ts = 0.2 ms (5 kHz), 与速度环 10 分频错峰
SMRC_I_k_lead      = int32(1);       % 输出路径相位超前补偿拍数 (1 拍 = 0.2 ms)
SMRC_I_dx_min_mm   = single(0.01);   % 最低有效位移门限 (mm/步, 对应 50 mm/s; 20/0.01=2000 < N_BUF=2048)
SMRC_I_xf_limit_A  = single(0.35);   % 内部记忆状态限幅 (A)
SMRC_I_Vsat_thresh = single(0.998 * 0.8 * inverter.V_dc * sqrt(3) / 3); % 电压饱和抗饱和门限 (~11.06 V, 仅实际触发11.085V限幅时冻结)