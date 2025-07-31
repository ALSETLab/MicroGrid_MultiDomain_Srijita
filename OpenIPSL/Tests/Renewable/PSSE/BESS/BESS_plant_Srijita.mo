within OpenIPSL.Tests.Renewable.PSSE.BESS;
model BESS_plant_Srijita
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
  Electrical.Renewables.PSSE.RenewableElectricalController.REECDU1 rEECDU1_1 annotation (Placement(transformation(extent={{-172,-32},{-112,28}})));
  Electrical.Renewables.PSSE.RenewableGeneratorConverter.REGC_A rEGC_A annotation (Placement(transformation(extent={{-86,-18},{-52,18}})));
  Modelica.Blocks.Sources.Constant PAUX(k=0)
    annotation (Placement(transformation(extent={{-226,-22},{-216,-12}})));
  Modelica.Blocks.Sources.Constant WG(k=0) annotation (Placement(transformation(extent={{-230,-2},{-220,8}})));
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
  connect(rEGC_A.p, GEN1.p) annotation (Line(points={{-52,0},{-30,0}}, color={0,0,255}));
  connect(rEECDU1_1.Iqcmd, rEGC_A.Iqcmd) annotation (Line(points={{-111.189,10.1875},{-98,10.1875},{-98,9},{-88.4286,9}},
                                                                                                            color={0,0,127}));
  connect(rEECDU1_1.Ipcmd, rEGC_A.Ipcmd) annotation (Line(points={{-111.189,-18.125},{-96,-18.125},{-96,-9},{-88.4286,-9}},
                                                                                                                color={0,0,127}));
  connect(rEECDU1_1.iq0, rEGC_A.IQ0) annotation (Line(points={{-117.676,-33.875},{-117.676,-44},{-83.5714,-44},{-83.5714,-19.2857}},
                                                                                                                         color={0,0,127}));
  connect(rEECDU1_1.ip0, rEGC_A.IP0) annotation (Line(points={{-126.595,-33.875},{-126.595,-50},{-77.9857,-50},{-77.9857,-19.2857}},
                                                                                                                         color={0,0,127}));
  connect(rEECDU1_1.v0, rEGC_A.V_0) annotation (Line(points={{-136.324,-33.875},{-136.324,-54},{-71.6714,-54},{-71.6714,-19.2857}},
                                                                                                              color={0,0,127}));
  connect(rEECDU1_1.q0, rEGC_A.q_0) annotation (Line(points={{-146.054,-33.875},{-146.054,-60},{-59.2857,-60},{-59.2857,-19.2857}},
                                                                                                                        color={0,0,127}));
  connect(rEECDU1_1.p0, rEGC_A.p_0) annotation (Line(points={{-154.973,-33.875},{-154.973,-64},{-54.4286,-64},{-54.4286,-19.2857}},
                                                                                                                        color={0,0,127}));
  connect(rEECDU1_1.Vt, rEGC_A.V_t) annotation (Line(points={{-173.622,24.625},{-184,24.625},{-184,34},{-82.8429,34},{-82.8429,19.2857}},
                                                                                                                              color={0,0,127}));
  connect(rEGC_A.Pgen,rEECDU1_1. Pe) annotation (Line(points={{-64.1429,19.2857},{-64.1429,40},{-194,40},{-194,9.4375},{-173.622,9.4375}},
                                                                                                                     color={0,0,127}));
  connect(rEGC_A.Qgen,rEECDU1_1. Qgen) annotation (Line(points={{-58.0714,19.2857},{-58.0714,46},{-204,46},{-204,1.5625},{-173.622,1.5625}},
                                                                                                                               color={0,0,127}));
  connect(rEECDU1_1.Qext, rEGC_A.q_0) annotation (Line(points={{-173.622,-5.9375},{-204,-5.9375},{-204,-60},{-59.2857,-60},{-59.2857,-19.2857}},
                                                                                                                                   color={0,0,127}));
  connect(rEECDU1_1.Pref, rEGC_A.p_0) annotation (Line(points={{-173.622,-13.25},{-182,-13.25},{-182,-64},{-54.4286,-64},{-54.4286,-19.2857}},
                                                                                                                                     color={0,0,127}));
  connect(PAUX.y,rEECDU1_1. Paux) annotation (Line(points={{-215.5,-17},{-215.5,-26.9375},{-173.622,-26.9375}},
                                                                                                  color={0,0,127}));
  connect(WG.y, rEECDU1_1.Wg) annotation (Line(points={{-219.5,3},{-210,3},{-210,-20.375},{-173.622,-20.375}}, color={0,0,127}));
  connect(rEECDU1_1.It, rEGC_A.I_t) annotation (Line(points={{-173.622,17.3125},{-208,17.3125},{-208,56},{-76.2857,56},{-76.2857,19.2857}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{100,100}})),
                                                                 Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-240,-100},{100,100}})));
end BESS_plant_Srijita;
