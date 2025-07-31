within OpenIPSL.Tests.Renewable.PSSE.Wind;
model Wind_plant_Srijita
  extends Modelica.Icons.Example;
  OpenIPSL.Electrical.Branches.PwLine pwLine(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{34,10},{54,30}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine1(
    R=2.50000E-2,
    X=2.50000E-2,
    G=0,
    B=0.05000/2) annotation (Placement(transformation(extent={{34,-30},{54,-10}})));
  OpenIPSL.Electrical.Machines.PSSE.GENCLS gENCLS2_1(
    angle_0=-1.570655e-05,
    R_a=0,
    X_d=2.00000E-1,
    M_b=100000000,
    V_b=100000,
    P_0=-1498800,
    Q_0=-4334000,
    v_0=1.00000) annotation (Placement(transformation(extent={{98,-12},{86,12}})));
  OpenIPSL.Electrical.Branches.PwLine pwLine2(
    G=0,
    B=0,
    R=2.50000E-3,
    X=2.50000E-3)
    annotation (Placement(transformation(extent={{-6,-10},{14,10}})));
  OpenIPSL.Electrical.Events.PwFault pwFault(
    R=0.5,
    X=0.5,
    t1=2.00,
    t2=2.15)
            annotation (Placement(transformation(extent={{32,-60},{52,-40}})));
  inner OpenIPSL.Electrical.SystemBase SysData(fn=50, S_b=100000000) annotation (Placement(transformation(extent={{-100,80},
            {-60,100}})));
  OpenIPSL.Electrical.Buses.Bus GEN1
    annotation (Placement(transformation(extent={{-40,-10},{-20,10}})));
  OpenIPSL.Electrical.Buses.Bus FAULT
    annotation (Placement(transformation(extent={{6,-10},{26,10}})));
  OpenIPSL.Electrical.Buses.Bus GEN2
    annotation (Placement(transformation(extent={{64,-10},{84,10}})));
  Electrical.Sensors.PwCurrent pwCurrent
    annotation (Placement(transformation(extent={{-20,-6},{-8,6}})));
  Electrical.Sensors.PwVoltage pwVoltage annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={-30,30})));
  Electrical.Renewables.PSSE.RenewableElectricalController.REECDU1 rEECDU1_1
                                                                           annotation (Placement(transformation(extent={{-166,-22},{-112,32}})));
  Electrical.Renewables.PSSE.RenewableGeneratorConverter.REGC_A rEGC_A annotation (Placement(transformation(extent={{-94,-20},{-50,20}})));
  Electrical.Renewables.PSSE.RenewableDriveTrain.WTDTA1 wTDTA1_1 annotation (Placement(transformation(extent={{-208,-74},{-184,-50}})));
  Modelica.Blocks.Sources.Constant w0(k=0)
    annotation (Placement(transformation(extent={{-136,-82},{-146,-72}})));
  //parameter OpenIPSL.Types.PerUnit W0(fixed=false);
  Modelica.Blocks.Sources.Constant PAUX(k=0) annotation (Placement(transformation(extent={{-216,-28},{-206,-18}})));
equation
  connect(FAULT.p,pwLine. p)
    annotation (Line(points={{16,0},{24,0},{24,20},{35,20}}, color={0,0,255}));
  connect(pwLine1.p,pwLine. p) annotation (Line(points={{35,-20},{24,-20},{24,20},
          {35,20}},              color={0,0,255}));
  connect(pwFault.p,FAULT. p) annotation (Line(points={{30.3333,-50},{20,-50},{20,0},{16,0}},
                         color={0,0,255}));
  connect(pwLine.n,GEN2. p)
    annotation (Line(points={{53,20},{64,20},{64,0},{74,0}}, color={0,0,255}));
  connect(pwLine1.n,GEN2. p) annotation (Line(points={{53,-20},{64,-20},{64,0},{
          74,0}},  color={0,0,255}));
  connect(GEN2.p,gENCLS2_1. p)
    annotation (Line(points={{74,0},{86,0}},        color={0,0,255}));
  connect(pwLine2.n, FAULT.p)
    annotation (Line(points={{13,0},{16,0}}, color={0,0,255}));
  connect(pwCurrent.n, pwLine2.p)
    annotation (Line(points={{-8,0},{-5,0}}, color={0,0,255}));
  connect(pwCurrent.p, GEN1.p)
    annotation (Line(points={{-20,0},{-30,0}}, color={0,0,255}));
  connect(pwVoltage.p, GEN1.p) annotation (Line(points={{-24,30},{-22,30},{-22,
          0},{-30,0}}, color={0,0,255}));
  connect(rEECDU1_1.Iqcmd, rEGC_A.Iqcmd) annotation (Line(points={{-111.27,15.9688},{-106,15.9688},{-106,10},{-97.1429,10}}, color={0,0,127}));
  connect(rEECDU1_1.Ipcmd, rEGC_A.Ipcmd) annotation (Line(points={{-111.27,-9.5125},{-111.27,-10},{-97.1429,-10}}, color={0,0,127}));
  connect(rEECDU1_1.iq0, rEGC_A.IQ0) annotation (Line(points={{-117.108,-23.6875},{-117.108,-32},{-90.8571,-32},{-90.8571,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.ip0, rEGC_A.IP0) annotation (Line(points={{-125.135,-23.6875},{-125.135,-36},{-81.4286,-36},{-81.4286,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.v0, rEGC_A.V_0) annotation (Line(points={{-133.892,-23.6875},{-133.892,-40},{-72,-40},{-72,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.q0, rEGC_A.q_0) annotation (Line(points={{-142.649,-23.6875},{-142.649,-46},{-62.5714,-46},{-62.5714,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.p0, rEGC_A.p_0) annotation (Line(points={{-150.676,-23.6875},{-150.676,-50},{-53.1429,-50},{-53.1429,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Vt, rEGC_A.V_t) annotation (Line(points={{-167.459,28.9625},{-178,28.9625},{-178,34},{-86.1429,34},{-86.1429,21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Pe, rEGC_A.Pgen) annotation (Line(points={{-167.459,15.2938},{-192,15.2938},{-192,42},{-72,42},{-72,21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Qgen, rEGC_A.Qgen) annotation (Line(points={{-167.459,8.20625},{-198,8.20625},{-198,46},{-57.8571,46},{-57.8571,21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Qext, rEGC_A.q_0) annotation (Line(points={{-167.459,1.45625},{-180,1.45625},{-180,-46},{-62.5714,-46},{-62.5714,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Pref, rEGC_A.p_0) annotation (Line(points={{-167.459,-5.125},{-174,-5.125},{-174,-42},{-159.7,-42},{-159.7,-50},{-53.1429,-50},{-53.1429,-21.4286}}, color={0,0,127}));
  connect(wTDTA1_1.Pm, rEGC_A.p_0) annotation (Line(points={{-210,-56},{-228,-56},{-228,-96},{-54,-96},{-54,-50},{-53.1429,-50},{-53.1429,-21.4286}}, color={0,0,127}));
  connect(wTDTA1_1.Pe, rEGC_A.Pgen) annotation (Line(points={{-210,-68},{-222,-68},{-222,-90},{-42,-90},{-42,42},{-72,42},{-72,21.4286}}, color={0,0,127}));
  connect(wTDTA1_1.P0, rEGC_A.p_0) annotation (Line(points={{-190,-76},{-190,-96},{-54,-96},{-54,-50},{-53.1429,-50},{-53.1429,-21.4286}}, color={0,0,127}));
  connect(rEECDU1_1.Wg, wTDTA1_1.wg) annotation (Line(points={{-167.459,-11.5375},{-178,-11.5375},{-178,-68},{-183,-68}}, color={0,0,127}));
  connect(w0.y, wTDTA1_1.W_0) annotation (Line(points={{-146.5,-77},{-176,-77},{-176,-84},{-202,-84},{-202,-76}}, color={0,0,127}));
  connect(rEGC_A.p, GEN1.p) annotation (Line(points={{-50,0},{-30,0}}, color={0,0,255}));
  connect(PAUX.y, rEECDU1_1.Paux) annotation (Line(points={{-205.5,-23},{-205.5,-24},{-172,-24},{-172,-17.4438},{-167.459,-17.4438}}, color={0,0,127}));
  connect(rEECDU1_1.It, rEECDU1_1.Paux) annotation (Line(points={{-167.459,22.3813},{-190,22.3813},{-190,-24},{-172,-24},{-172,-17.4438},{-167.459,-17.4438}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{100,100}})),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{100,100}})));
end Wind_plant_Srijita;
