within OpenIPSL.Electrical.Renewables.PSSE;
model PV_NewREGCB
  "Framework for a photovoltaic plant including controllers"
extends OpenIPSL.Electrical.Essentials.pfComponent(
    final enablefn=false,
    final enableV_b=false,
    final enableangle_0=true,
    final enablev_0=true,
    final enableQ_0=true,
    final enableP_0=true,
    final enabledisplayPF=true,
    final enableS_b=true);

  // Parameters for selection
  parameter Integer QFunctionality = 0 annotation (Dialog(group= "Reactive Power Control Options"), choices(choice=0 "Constant local PF control", choice=1 "Constant local Q control", choice=2 "Local V control", choice=3 "Local coordinated V/Q control", choice=4 "Plant level Q control", choice=5 "Plant level V control", choice=6 "Plant level Q control + local coordinated V/Q control", choice=7 "Plant level V control + local coordinated V/Q control"));
  parameter Integer PFunctionality = 0 annotation (Dialog(group= "Active Power Control Options", enable=(QFunctionality >=4)), choices(choice=0 "No governor response",  choice=1 "Governor response with up and down regulation"));

  replaceable RenewableGeneratorConverter.BaseClasses.baseREGC_B
    RenewableGenerator(
    P_0=P_0,
    Q_0=Q_0,
    v_0=v_0,
    angle_0=angle_0,
    rflag=rflag,
    pqflag=pqflag)
    annotation (choicesAllMatching=true,Placement(transformation(extent={{30,-20},
            {70,20}})));
  Interfaces.PwPin pwPin
    annotation (Placement(transformation(extent={{90,-10},{110,10}})));
  replaceable RenewablePlantController.BaseClasses.basePlantController
    PlantController(
    P_0=P_0,
    Q_0=Q_0,
    v_0=v_0,
    angle_0=angle_0,
    fflag=fflag,
    refflag=refflag) if QFunctionality >= 4 annotation (choicesAllMatching=true,
      Placement(transformation(extent={{-82,-20},{-42,20}})));
  Modelica.Blocks.Math.Gain gain(k=1)
                                 if QFunctionality < 4
    annotation (Placement(transformation(
        extent={{-6,-6},{6,6}},
        rotation=180,
        origin={-6,-60})));
  Modelica.Blocks.Math.Gain gain1(k=1)
                                  if QFunctionality < 4
    annotation (Placement(transformation(
        extent={{-6,6},{6,-6}},
        rotation=180,
        origin={-6,-80})));
  Modelica.Blocks.Sources.Constant freq_ref(k=SysData.fn) if QFunctionality >= 4
    annotation (Placement(transformation(extent={{-70,-60},{-80,-50}})));
  Modelica.Blocks.Interfaces.RealInput FREQ if QFunctionality >= 4
    annotation (Placement(transformation(extent={{-126,-20},{-86,20}}),
        iconTransformation(extent={{-126,-20},{-86,20}})));

  Modelica.Blocks.Interfaces.RealInput branch_ir if QFunctionality >= 4
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-70,80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={60,-86})));
  Modelica.Blocks.Interfaces.RealInput branch_ii if QFunctionality >= 4
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-30,80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=90,
        origin={-60,-86})));
  Modelica.Blocks.Interfaces.RealInput regulate_vr if QFunctionality >= 4
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={30,80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={-60,86})));
  Modelica.Blocks.Interfaces.RealInput regulate_vi if QFunctionality >= 4
    annotation (Placement(transformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={70,80}), iconTransformation(
        extent={{-20,-20},{20,20}},
        rotation=270,
        origin={60,86})));
  replaceable OpenIPSL.Electrical.Renewables.PSSE.RenewableElectricalController.BaseClasses.BaseREECD
  RenewableController(
  pfflag=pfflag,
  vflag=vflag,
  qflag=qflag,
  pqflag=false,
  pflag=false,
  vcmpflag=vcmpflag) annotation (Placement(transformation(extent={{-22,-18},{12,18}})));
  Modelica.Blocks.Sources.Constant WG(k=0) annotation (Placement(transformation(extent={{-62,-82},{-52,-72}})));
  Modelica.Blocks.Sources.Constant PAUX(k=0) annotation (Placement(transformation(extent={{-62,-64},{-52,-54}})));
protected
      parameter Boolean pfflag =  (if QFunctionality == 0 then true else false);
      parameter Boolean vflag =   (if QFunctionality == 3 or QFunctionality == 6 or QFunctionality == 7 then true else false);
      parameter Boolean qflag =   (if QFunctionality == 2 or QFunctionality == 3 or QFunctionality == 6 or QFunctionality == 7 then true else false);
      //parameter Boolean refflag = (if QFunctionality == 5 or QFunctionality == 7 then true else false);
      //parameter Boolean fflag = (if PFunctionality == 1 then true else false);
      parameter Boolean vcmpflag = (if QFunctionality == 0 then true else false);
      //parameter Boolean rflag = (if QFunctionality == 0 then true else false);
equation
  connect(gain.u, RenewableGenerator.p_0) annotation (Line(points={{1.2,-60},{
          67.1429,-60},{67.1429,-21.4286}},
                                    color={0,0,127}));
  connect(gain1.u, RenewableGenerator.q_0) annotation (Line(points={{1.2,-80},{
          58.5714,-80},{58.5714,-21.4286}},
                                    color={0,0,127}));
  connect(freq_ref.y, PlantController.Freq_ref) annotation (Line(points={{-80.5,
          -55},{-88,-55},{-88,-11},{-84,-11}}, color={0,0,127}));
  connect(PlantController.Plant_pref, RenewableGenerator.p_0) annotation (Line(
        points={{-84,4},{-96,4},{-96,-92},{67.1429,-92},{67.1429,-21.4286}},
        color={0,0,127}));
  connect(PlantController.Qref, RenewableGenerator.q_0) annotation (Line(points={{-84,11},
          {-98,11},{-98,-98},{58.5714,-98},{58.5714,-21.4286}},          color={
          0,0,127}));
  connect(RenewableGenerator.p, pwPin)
    annotation (Line(points={{70,0},{100,0}}, color={0,0,255}));
  connect(PlantController.Freq, FREQ) annotation (Line(points={{-84,-4},{-92,-4},
          {-92,0},{-106,0}}, color={0,0,127}));
  connect(PlantController.branch_ii, branch_ii) annotation (Line(points={{-66,22},
          {-66,58},{-30,58},{-30,80}},     color={0,0,127}));
  connect(PlantController.branch_ir, branch_ir) annotation (Line(points={{-76,22},
          {-76,56},{-70,56},{-70,80}},     color={0,0,127}));
  connect(PlantController.regulate_vr, regulate_vr) annotation (Line(points={{-56,22},
          {-56,54},{30,54},{30,80}},     color={0,0,127}));
  connect(PlantController.regulate_vi, regulate_vi) annotation (Line(points={{-48,22},
          {-48,50},{70,50},{70,80}},     color={0,0,127}));
  connect(PlantController.p0, RenewableGenerator.p_0) annotation (Line(points={{-74,-22},
          {-74,-42},{67.1429,-42},{67.1429,-21.4286}},                                                                                color={0,0,127}));
  connect(RenewableGenerator.Ipcmd, RenewableController.Ipcmd) annotation (Line(
        points={{27.1429,-10},{28,-9.675},{12.4595,-9.675}}, color={0,0,127}));
  connect(RenewableGenerator.Iqcmd, RenewableController.Iqcmd) annotation (Line(
        points={{27.1429,10},{16,10},{16,7.3125},{12.4595,7.3125}}, color={0,0,
          127}));
  connect(RenewableGenerator.V_t, RenewableController.Vt) annotation (Line(
        points={{35.1429,21.4286},{35.1429,26},{-26,26},{-26,15.975},{-22.9189,
          15.975}}, color={0,0,127}));
  connect(RenewableGenerator.I_t, RenewableController.It) annotation (Line(
        points={{41.4286,21.4286},{41.4286,30},{-28,30},{-28,11.5875},{-22.9189,
          11.5875}}, color={0,0,127}));
  connect(RenewableGenerator.Pgen, RenewableController.Pe) annotation (Line(
        points={{58.5714,21.4286},{58.5714,32},{-30,32},{-30,6.8625},{-22.9189,
          6.8625}}, color={0,0,127}));
  connect(RenewableGenerator.Qgen, RenewableController.Qgen) annotation (Line(
        points={{65.7143,21.4286},{65.7143,36},{-32,36},{-32,2.1375},{-22.9189,
          2.1375}}, color={0,0,127}));
  connect(RenewableGenerator.IQ0, RenewableController.iq0) annotation (Line(
        points={{32.8571,-21.4286},{32.8571,-26},{8.78378,-26},{8.78378,-19.125}},
        color={0,0,127}));
  connect(RenewableGenerator.IP0, RenewableController.ip0) annotation (Line(
        points={{41.4286,-21.4286},{41.4286,-28},{3.72973,-28},{3.72973,-19.125}},
        color={0,0,127}));
  connect(RenewableGenerator.V_0, RenewableController.v0) annotation (Line(
        points={{50,-21.4286},{50,-32},{-1.78378,-32},{-1.78378,-19.125}},
                     color={0,0,127}));
  connect(PlantController.v0, RenewableGenerator.V_0) annotation (Line(points={{-62,-22},
          {-62,-32},{50,-32},{50,-21.4286}},                     color={0,0,127}));
  connect(RenewableController.q0, RenewableGenerator.q_0) annotation (Line(
        points={{-7.2973,-19.125},{-7.2973,-34},{61.4286,-34},{61.4286,-21.4286},
          {58.5714,-21.4286}}, color={0,0,127}));
  connect(PlantController.q0, RenewableGenerator.q_0) annotation (Line(points={{-50,-22},
          {-50,-34},{58.5714,-34},{58.5714,-21.4286}},           color={0,0,127}));
  connect(RenewableController.p0, RenewableGenerator.p_0) annotation (Line(
        points={{-12.3514,-19.125},{-12.3514,-42},{67.143,-42},{67.143,-21.4286},
          {67.1429,-21.4286}}, color={0,0,127}));
  connect(WG.y, RenewableController.Wg) annotation (Line(points={{-51.5,-77},{
          -32,-77},{-32,-11.025},{-22.9189,-11.025}}, color={0,0,127}));
  connect(PAUX.y, RenewableController.Paux) annotation (Line(points={{-51.5,-59},
          {-26,-59},{-26,-14.9625},{-22.9189,-14.9625}}, color={0,0,127}));
  connect(PlantController.Qext, RenewableController.Qext) annotation (Line(
        points={{-41,10},{-36,10},{-36,-2.3625},{-22.9189,-2.3625}}, color={0,0,
          127}));
  connect(gain1.y, RenewableController.Qext) annotation (Line(points={{-12.6,
          -80},{-36,-80},{-36,-2.3625},{-22.9189,-2.3625},{-22.9189,-2.3625}},
        color={0,0,127}));
  connect(PlantController.Pref, RenewableController.Pref) annotation (Line(
        points={{-41,-10},{-34,-10},{-34,-6.75},{-22.9189,-6.75}}, color={0,0,
          127}));
  connect(gain.y, RenewableController.Pref) annotation (Line(points={{-12.6,-60},
          {-20,-60},{-20,-48},{-38,-48},{-38,-10},{-34,-10},{-34,-6.75},{
          -22.9189,-6.75}}, color={0,0,127}));
  connect(RenewableController.i0, RenewableGenerator.I_0) annotation (Line(
        points={{0.972973,-19.0125},{0.972973,-30},{54.8571,-30},{54.8571,
          -21.4286}},
        color={0,0,127}));
  annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-120,
            -120},{120,100}}),                                  graphics={
                                   Ellipse(
          extent={{-100,100},{100,-100}},
          lineColor={0,0,0},
          fillColor={255,255,170},
          fillPattern=FillPattern.Solid),
                         Text(
          extent={{-40,20},{40,-20}},
          lineColor={0,0,0},
          textString="%name"),           Line(
          points={{-20,20},{-44,42},{-66,32},{-80,0}},
          color={0,0,0},
          smooth=Smooth.Bezier), Line(
          points={{20,-20},{44,-42},{66,-32},{80,0}},
          color={0,0,0},
          smooth=Smooth.Bezier)}),                               Diagram(
        coordinateSystem(preserveAspectRatio=false, extent={{-120,-120},{120,
            100}})),
    Documentation(info="<html>
<p>
This model is meant as a simple framework to create a photovoltaic power plant that consists of:
</p>
<ul>
<li>Generator/Converter</li>
<li>Electrical Controller</li>
<li>Plant Controller</li>
</ul>
<p>
The type of each can be selected via a drop down list where also
 a deactivation is provided (normally via feed through). 
</p>
<p>
The type of control configuration can also be selected via drop down list.
</html>"));
end PV_NewREGCB;
