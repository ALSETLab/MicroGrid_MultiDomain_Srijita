within OpenIPSL.Electrical.Renewables.PSSE.Figures;
model REGC_B "Renewable energy generator/converter model B"
  extends RenewableGeneratorConverter.BaseClasses.baseREGC_B(
    break V_0,
    break q_0,
    break p_0,
    break V_t,
    break Pgen,
    break Qgen,
    break IP0,
    break IQ0);
  OpenIPSL.NonElectrical.Continuous.SimpleLag simpleLag(
    K=1,
    T=Tfltr,
    y_start=v_0)
    annotation (Placement(transformation(extent={{36,-98},{16,-78}})));
  Modelica.Blocks.Nonlinear.Limiter limiter4(uMax=rrpwr, uMin=-Modelica.Constants.inf)
    annotation (Placement(transformation(extent={{-44,-36},{-24,-16}})));
  Modelica.Blocks.Math.Add add2(k2=-1)
    annotation (Placement(transformation(extent={{-80,-36},{-60,-16}})));
  Modelica.Blocks.Sources.RealExpression Vt(y=VT) annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=0,
        origin={116,-88})));
  Modelica.Blocks.Math.Add add3(k2=-1)
    annotation (Placement(transformation(extent={{-96,66},{-76,86}})));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=Iqrmax, uMin=Iqrmin)
    annotation (Placement(transformation(extent={{-66,64},{-46,84}})));
  Modelica.Blocks.Continuous.Integrator integrator(
    k=1/Tg,
    initType=Modelica.Blocks.Types.Init.InitialState,
    y_start=Iq0)
    annotation (Placement(transformation(extent={{-34,64},{-14,84}})));
  Modelica.Blocks.Math.Gain gain(k=-1)
    annotation (Placement(transformation(extent={{-128,70},{-108,90}})));
  Modelica.Blocks.Continuous.Integrator integrator1(
    k=1/Tg,
    initType=Modelica.Blocks.Types.Init.InitialState,
    y_start=Ip0)
    annotation (Placement(transformation(extent={{-10,-36},{10,-16}})));
  RenewableGeneratorConverter.BaseClasses.Thevenin thevenin(Re=Re, Xe=Xe)
    annotation (Placement(transformation(extent={{14,54},{50,90}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag Eq(
    K=1,
    T=Te,
    y_start=v_0) annotation (Placement(transformation(extent={{82,88},{102,108}})));
  OpenIPSL.NonElectrical.Continuous.SimpleLag Ed(
    K=1,
    T=Te,
    y_start=v_0) annotation (Placement(transformation(extent={{82,44},{102,64}})));
  Modelica.Blocks.Math.Division division annotation (Placement(transformation(extent={{38,-42},{58,-22}})));
  Modelica.Blocks.Math.Product product1 annotation (Placement(transformation(extent={{-118,-30},{-98,-10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter2(uMax=Modelica.Constants.inf,
                                                      uMin=0.01)
    annotation (Placement(transformation(extent={{0,-98},{-20,-78}})));
  Modelica.Blocks.Logical.Switch RateFlag annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        rotation=270,
        origin={-56,-82})));
  Modelica.Blocks.Sources.BooleanConstant RFLAG(k=rflag)
    annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-56,-122})));
  Modelica.Blocks.Sources.Constant const(k=1.0) annotation (Placement(transformation(extent={{-10,-10},{10,10}},
        rotation=0,
        origin={-100,-102})));

  import Modelica.ComplexMath.j;

  //parameter Types.PerUnit Re=0 "Resistance";
  //parameter Types.PerUnit Xe=0.05 "Reactance";
  //parameter Real Vterminal_Re "Real part of terminal voltage";
  //parameter Real Vterminal_Im "Imaginary part of terminal voltage";

  //parameter Types.PerUnit Re=0 "Generator Effective Resistance(0-0.01 pu)";
  //parameter Types.PerUnit Xe=0.5 "Generator Effective Reactance(0.05-0.2 pu)";
  Modelica.Blocks.Interfaces.RealOutput Qgen "Value of Real output" annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={48,164}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,150})));
  Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output" annotation (
     Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-34,164}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,150})));
  Modelica.Blocks.Interfaces.RealOutput V_t "Value of Real output" annotation (
      Placement(transformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={-118,164}), iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=90,
        origin={90,150})));
  Modelica.Blocks.Interfaces.RealOutput IP0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={10,-176}),  iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-150})));
  Modelica.Blocks.Interfaces.RealOutput IQ0 annotation (Placement(
        transformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={76,-180}),  iconTransformation(
        extent={{-10,-10},{10,10}},
        rotation=270,
        origin={-60,-150})));
  Modelica.Blocks.Sources.RealExpression TerminalVoltage(y=VT) annotation (
      Placement(transformation(
        extent={{21,-11},{-21,11}},
        rotation=0,
        origin={-87,125})));
  Modelica.Blocks.Sources.RealExpression ActivePower(y=-(1/CoB)*(p.vr*p.ir + p.vi
        *p.ii)) annotation (Placement(transformation(
        extent={{21,-11},{-21,11}},
        rotation=0,
        origin={-1,123})));
  Modelica.Blocks.Sources.RealExpression ReactivePower(y=-(1/CoB)*(p.vi*p.ir -
        p.vr*p.ii)) annotation (Placement(transformation(
        extent={{21,-11},{-21,11}},
        rotation=0,
        origin={83,127})));
  Modelica.Blocks.Sources.RealExpression Initial_P0(y=P0) annotation (Placement(
        transformation(
        extent={{21,-11},{-21,11}},
        rotation=0,
        origin={41,-131})));
  Modelica.Blocks.Sources.RealExpression Initial_Q0(y=Q0) annotation (Placement(
        transformation(
        extent={{21,-11},{-21,11}},
        rotation=0,
        origin={107,-133})));
equation

  V_t = VT;
  //I_t = IT;

  // Convert terminal voltage to real and imaginary parts
  //Vterminal_Re = VT * cos(delta);
  //Vterminal_Im = VT * sin(delta);
  [p.ir; p.ii] = -[Modelica.ComplexMath.real(Icom.y); Modelica.ComplexMath.imag(Icom.y)];

  Pgen = -(1/CoB)*(p.vr*p.ir + p.vi*p.ii);
  Qgen = -(1/CoB)*(p.vi*p.ir - p.vr*p.ii);
  IQ0 = Iq0;
  IP0 = Ip0;
  V_0 = v_0;
  p_0 = p0;
  q_0 = q0;

  connect(add2.y,limiter4. u)
    annotation (Line(points={{-59,-26},{-46,-26}}, color={0,0,127}));
  connect(limiter1.y,integrator. u)
    annotation (Line(points={{-45,74},{-36,74}},
                                               color={0,0,127}));
  connect(gain.u, Iqcmd) annotation (Line(points={{-130,80},{-160,80}},
                      color={0,0,127}));
  connect(limiter4.y, integrator1.u)
    annotation (Line(points={{-23,-26},{-12,-26}}, color={0,0,127}));
  connect(gain.y, add3.u1) annotation (Line(points={{-107,80},{-106,80},{-106,82},{-98,82}},          color={0,0,127}));
  connect(add3.y, limiter1.u) annotation (Line(points={{-75,76},{-75,74},{-68,74}}, color={0,0,127}));
  connect(integrator.y, thevenin.Iqneg) annotation (Line(points={{-13,74},{-8,74},{-8,84.24},{10.4,84.24}}, color={0,0,127}));
  connect(add3.u2, thevenin.Iqneg) annotation (Line(points={{-98,70},{-104,70},{-104,54},{-6,54},{-6,74},{-8,74},{-8,84.24},{10.4,84.24}}, color={0,0,127}));
  connect(Vt.y, simpleLag.u) annotation (Line(points={{105,-88},{38,-88}}, color={0,0,127}));
  connect(thevenin.V, simpleLag.u) annotation (Line(points={{10.4,59.04},{4,59.04},{4,36},{90,36},{90,-88},{38,-88}}, color={0,0,127}));
  connect(division.y, thevenin.Ip) annotation (Line(points={{59,-32},{80,-32},{80,20},{0,20},{0,72},{10.4,72}}, color={0,0,127}));
  connect(thevenin.Edcalc, Ed.u) annotation (Line(points={{53.24,65.52},{70,
          65.52},{70,54},{80,54}},                                                                   color={0,0,127}));
  connect(Ipcmd, product1.u1) annotation (Line(points={{-160,-60},{-134,-60},{-134,-14},{-120,-14}},
                                                                                                 color={0,0,127}));
  connect(product1.y, add2.u1) annotation (Line(points={{-97,-20},{-82,-20}},                                             color={0,0,127}));
  connect(limiter2.u, simpleLag.y) annotation (Line(points={{2,-88},{15,-88}}, color={0,0,127}));
  connect(integrator1.y, division.u1) annotation (Line(points={{11,-26},{36,-26}}, color={0,0,127}));
  connect(RFLAG.y, RateFlag.u2) annotation (Line(points={{-56,-111},{-56,-94}}, color={255,0,255}));
  connect(limiter2.y, RateFlag.u1) annotation (Line(points={{-21,-88},{-28,-88},{-28,-106},{-48,-106},{-48,-94}}, color={0,0,127}));
  connect(add2.u2, division.u1) annotation (Line(points={{-82,-32},{-88,-32},{-88,-46},{18,-46},{18,-26},{36,-26}}, color={0,0,127}));
  connect(RateFlag.y, product1.u2) annotation (Line(points={{-56,-71},{-56,-56},{-126,-56},{-126,-26},{-120,-26}}, color={0,0,127}));
  connect(division.u2, product1.u2) annotation (Line(points={{36,-38},{26,-38},{26,-56},{-126,-56},{-126,-26},{-120,-26}}, color={0,0,127}));
  connect(const.y, RateFlag.u3) annotation (Line(points={{-89,-102},{-64,-102},{-64,-94}}, color={0,0,127}));
  connect(TerminalVoltage.y, V_t) annotation (Line(points={{-110.1,125},{-118,
          125},{-118,164}}, color={0,0,127}));
  connect(thevenin.Eqcalc, Eq.u) annotation (Line(points={{53.24,80.28},{68,
          80.28},{68,98},{80,98}}, color={0,0,127}));
  connect(Pgen, ActivePower.y) annotation (Line(points={{-34,164},{-34,123},{
          -24.1,123}}, color={0,0,127}));
  connect(ReactivePower.y, Qgen)
    annotation (Line(points={{59.9,127},{48,127},{48,164}}, color={0,0,127}));
  connect(Initial_P0.y, IP0) annotation (Line(points={{17.9,-131},{17.9,-132},{
          10,-132},{10,-176}}, color={0,0,127}));
  connect(Initial_Q0.y, IQ0) annotation (Line(points={{83.9,-133},{83.9,-134},{
          76,-134},{76,-180}}, color={0,0,127}));
  annotation (Icon(coordinateSystem(extent={{-140,-220},{140,200}}),
                   graphics={Text(
          extent={{-90,70},{90,-70}},
          textColor={0,0,255},
          textString="REGCB")}), Diagram(coordinateSystem(extent={{-140,-220},{
            140,200}})));
end REGC_B;
