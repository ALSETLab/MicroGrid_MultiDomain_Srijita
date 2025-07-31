within OpenIPSL.Electrical.Renewables.PSSE.RenewableGeneratorConverter;
package BaseClasses "Base classes for renewable energy generator/converter models from PSSE"
  extends Modelica.Icons.BasesPackage;
  partial model baseRenewableGenerator
    "Base renewable generator/converter for PSSE models"
    import Complex;
    import Modelica.ComplexMath.arg;
    import Modelica.ComplexMath.real;
    import Modelica.ComplexMath.imag;
    import Modelica.ComplexMath.conj;
    import Modelica.Blocks.Interfaces.*;
    parameter OpenIPSL.Types.ApparentPower M_b=SysData.S_b "Machine base power" annotation (Dialog(group="Power flow data"));
    extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enablefn=false,
      final enableV_b=false,
      final enableangle_0=true,
      final enablev_0=true,
      final enableQ_0=true,
      final enableP_0=true,
      final enableS_b=true);

    // Set of Model Parameters
    parameter Modelica.Units.SI.Time Tg = 0.02 "Converter time constant (s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit rrpwr = 10 "Low Voltage Power Logic (LVPL) ramp rate limit (pu/s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Brkpt = 0.9 "LVPL characteristic voltage 2 (pu)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Zerox = 0.5 "LVPL characteristic voltage 1 (pu)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Lvpl1 = 1.22 "LVPL gain (pu)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Volim = 1.2 "Voltage limit (pu) for high voltage reactive current management" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit lvpnt1 = 0.8 "High voltage point for low voltage active current management (pu)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit lvpnt0 = 0.4 "Low voltage point for low voltage active current management (pu)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Iolim = -1.3 "Current limit (pu) for high voltage reactive current management (specified as a negative value)" annotation (Dialog(group="Input Parameters"));
    parameter Modelica.Units.SI.Time Tfltr = 0.02 "Voltage filter time constant for low voltage active current management (s)" annotation (Dialog(group="Input Parameters"));
    parameter Real Khv = 0.7 "Overvoltage compensation gain used in the high voltage reactive current management" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Iqrmax = 9999 "Upper limit on rate of change for reactive current (pu/s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Iqrmin = -9999 "Lower limit on rate of change for reactive current (pu/s)" annotation (Dialog(group="Input Parameters"));

    parameter Boolean Lvplsw=true "Enable (True) or disable (False) low voltage power logic." annotation (Dialog(tab="Controls"));

    OpenIPSL.Interfaces.PwPin p(
      vr(start=vr0),
      vi(start=vi0),
      ir(start=-ir0*CoB),
      ii(start=-ii0*CoB)) annotation (Placement(transformation(extent={{130,-10},{150,10}}),
                            iconTransformation(extent={{130,-10},{150,10}})));
    Modelica.Blocks.Interfaces.RealInput Iqcmd(start=-Iq0)
      annotation (Placement(transformation(extent={{-180,60},{-140,100}}),
          iconTransformation(extent={{-180,50},{-140,90}})));
    Modelica.Blocks.Interfaces.RealInput Ipcmd(start=Ip0)
      annotation (Placement(transformation(extent={{-180,-80},{-140,-40}}),
          iconTransformation(extent={{-180,-90},{-140,-50}})));
    Modelica.Blocks.Interfaces.RealOutput IQ0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-120,-150}),iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-120,-150})));
    Modelica.Blocks.Interfaces.RealOutput IP0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-60,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-74,-150})));
    Modelica.Blocks.Interfaces.RealOutput V_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-20,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-22,-150})));
    Modelica.Blocks.Interfaces.RealOutput q_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={60,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={80,-150})));
    Modelica.Blocks.Interfaces.RealOutput p_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={120,-150})));
    Modelica.Blocks.Interfaces.RealOutput V_t annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-110,150}),iconTransformation(extent={{-10,-10},{10,10}},
            origin={-114,150},
          rotation=90)));
    Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={56,150}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={40,150})));
    Modelica.Blocks.Interfaces.RealOutput Qgen "Value of Real output"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={104,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={90,150})));
    Modelica.Blocks.Interfaces.RealOutput I_t annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-58,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-60,150})));
    Modelica.Blocks.Interfaces.RealOutput Angle_t annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={0,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-4,150})));
    Modelica.Blocks.Interfaces.RealOutput I_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={14,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={18,-150})));
  protected
    OpenIPSL.Types.Angle delta(start=angle_0);
    OpenIPSL.Types.PerUnit VT(start=v_0) "Bus voltage magnitude";
    OpenIPSL.Types.PerUnit IT(start=sqrt(ir0^2 + ii0^2)) "Terminal current magnitude";
    OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
    parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
    parameter OpenIPSL.Types.PerUnit vr0=v_0*cos(angle_0);
    parameter OpenIPSL.Types.PerUnit vi0=v_0*sin(angle_0);
    parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit Isr0=ir0 "Source current re M_b";
    parameter OpenIPSL.Types.PerUnit Isi0=ii0 "Source current im M_b";
    parameter OpenIPSL.Types.PerUnit Ip0=Isr0*cos(-angle_0) - Isi0*sin(-angle_0);
    parameter OpenIPSL.Types.PerUnit Iq0=(Isr0*sin(-angle_0) + cos(-angle_0)*Isi0);
    parameter Real CoB=M_b/S_b "Change of base";

  equation
    anglev = atan2(p.vi, p.vr);
    VT = sqrt(p.vr*p.vr + p.vi*p.vi);
    IT = sqrt(p.ii^2 + p.ir^2);
    delta = anglev;
    Angle_t = delta;

   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},
              {140,140}}), graphics={
          Rectangle(extent={{-140,140},{140,-140}}, lineColor={28,108,200}),
          Text(
            extent={{-126,90},{-66,50}},
            textColor={0,0,255},
            textString="IQCMD"),
          Text(
            extent={{-126,-50},{-66,-90}},
            textColor={0,0,255},
            textString="IPCMD"),
          Text(
            extent={{-136,-100},{-104,-140}},
            textColor={0,0,255},
            textString="IQ0"),
          Text(
            extent={{-88,-102},{-56,-142}},
            textColor={0,0,255},
            textString="IP0"),
          Text(
            extent={{-36,-108},{-12,-136}},
            textColor={0,0,255},
            textString="V0"),
          Text(
            extent={{72,-106},{94,-136}},
            textColor={0,0,255},
            textString="Q0"),
          Text(
            extent={{-130,138},{-90,108}},
            textColor={0,0,255},
            textString="VT"),
          Text(
            extent={{32,134},{72,112}},
            textColor={0,0,255},
            textString="PGEN"),
          Text(
            extent={{80,136},{120,110}},
            textColor={0,0,255},
            textString="QGEN"),
          Text(
            extent={{106,-104},{134,-136}},
            textColor={0,0,255},
            textString="P0"),
          Text(
            extent={{-74,136},{-34,106}},
            textColor={0,0,255},
            textString="IT"),
          Text(
            extent={{-20,136},{20,106}},
            textColor={0,0,255},
            textString="Angle"),
          Text(
            extent={{6,-110},{22,-136}},
            textColor={0,0,255},
            textString="I0")}),                                   Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
  end baseRenewableGenerator;

  partial model baseREGC_B "Base renewable generator/converter for PSSE models"
    import Complex;
    import Modelica.ComplexMath.j;
    import Modelica.ComplexMath.arg;
    import Modelica.ComplexMath.real;
    import Modelica.ComplexMath.imag;
    import Modelica.ComplexMath.conj;
    import Modelica.Blocks.Interfaces.*;
    parameter OpenIPSL.Types.ApparentPower M_b=SysData.S_b "Machine base power" annotation (Dialog(group="Power flow data"));
    extends OpenIPSL.Electrical.Essentials.pfComponent(
      final enabledisplayPF=false,
      final enablefn=false,
      final enableV_b=false,
      final enableangle_0=true,
      final enablev_0=true,
      final enableQ_0=true,
      final enableP_0=true,
      final enableS_b=true);

    // Set of Model Parameters
    parameter Modelica.Units.SI.Time Tg = 0.02 "Converter time constant (0.02-0.05s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit rrpwr = 10 "Low Voltage Power Logic (LVPL) ramp rate limit (1-20 pu/s)" annotation (Dialog(group="Input Parameters"));
    parameter Modelica.Units.SI.Time Tfltr = 0.02 "Voltage filter time constant for low voltage active current management (s)" annotation (Dialog(group="Input Parameters"));
    parameter Modelica.Units.SI.Time Te = 0 "Generator network impedance time constant(0-0.02s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Iqrmax = 9999 "Upper limit on rate of change for reactive current (pu/s)" annotation (Dialog(group="Input Parameters"));
    parameter OpenIPSL.Types.PerUnit Iqrmin = -9999 "Lower limit on rate of change for reactive current (pu/s)" annotation (Dialog(group="Input Parameters"));
    parameter Boolean rflag=true "Constant output value" annotation (Dialog(tab="Control"));
    parameter Types.PerUnit Re=0.01 "Generator Effective Resistance(0-0.01 pu)";
    parameter Types.PerUnit Xe=0.5 "Generator Effective Reactance(0.05-0.2 pu)";
    parameter Boolean pqflag=true;

    OpenIPSL.Interfaces.PwPin p(
      vr(start=vr0),
      vi(start=vi0),
      ir(start=-ir0*CoB),
      ii(start=-ii0*CoB)) annotation (Placement(transformation(extent={{130,-10},{150,10}}),
                            iconTransformation(extent={{130,-10},{150,10}})));
    Modelica.Blocks.Interfaces.RealInput Iqcmd(start=-Iq0)
      annotation (Placement(transformation(extent={{-180,60},{-140,100}}),
          iconTransformation(extent={{-180,50},{-140,90}})));
    Modelica.Blocks.Interfaces.RealInput Ipcmd(start=Ip0)
      annotation (Placement(transformation(extent={{-180,-80},{-140,-40}}),
          iconTransformation(extent={{-180,-90},{-140,-50}})));
    Modelica.Blocks.Interfaces.RealOutput IQ0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-120,-150}),iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-120,-150})));
    Modelica.Blocks.Interfaces.RealOutput IP0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-60,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={-60,-150})));
    Modelica.Blocks.Interfaces.RealOutput V_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={0,-150})));
    Modelica.Blocks.Interfaces.RealOutput q_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={60,-150})));
    Modelica.Blocks.Interfaces.RealOutput p_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={120,-150})));
    Modelica.Blocks.Interfaces.RealOutput V_t annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-116,150}),iconTransformation(extent={{-10,-10},{10,10}},
            origin={-104,150},
          rotation=90)));
    Modelica.Blocks.Interfaces.RealOutput Pgen "Value of Real output"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={60,150}),  iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={60,150})));
    Modelica.Blocks.Interfaces.RealOutput Qgen "Value of Real output"
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={102,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={110,150})));


    Modelica.Blocks.Interfaces.RealOutput I_t annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-50,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={-60,150})));
    Modelica.Blocks.Interfaces.RealOutput Angle_t annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={6,150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=90,
          origin={6,150})));
    Modelica.Blocks.Interfaces.RealOutput I_0 annotation (Placement(
          transformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={34,-150}), iconTransformation(
          extent={{-10,-10},{10,10}},
          rotation=270,
          origin={34,-150})));
  protected
    OpenIPSL.Types.Angle delta(start=angle_0);
    OpenIPSL.Types.PerUnit VT(start=v0) "Bus voltage magnitude";
    OpenIPSL.Types.PerUnit IT(start=sqrt(ir0^2 + ii0^2)) "Terminal current magnitude";
    OpenIPSL.Types.Angle anglev(start=angle_0) "Bus voltage angle";
    parameter OpenIPSL.Types.PerUnit p0=P_0/M_b "Initial active power (machine base)";
    parameter OpenIPSL.Types.PerUnit q0=Q_0/M_b "Initial reactive power (machine base)";
    parameter OpenIPSL.Types.PerUnit vr0 = Re * ir0 - Xe * ii0;
    parameter OpenIPSL.Types.PerUnit vi0 = Re * ii0 + Xe * ir0;
    parameter OpenIPSL.Types.PerUnit v0 = sqrt(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit ir0=(p0*vr0 + q0*vi0)/(vr0^2 + vi0^2);
    parameter OpenIPSL.Types.PerUnit ii0=(p0*vi0 - q0*vr0)/(vr0^2 + vi0^2);
    //parameter OpenIPSL.Types.PerUnit Isr0=ir0 "Source current re M_b";
    //parameter OpenIPSL.Types.PerUnit Isi0=ii0 "Source current im M_b";
    //parameter OpenIPSL.Types.PerUnit Ip0=Isr0*cos(-angle_0) - Isi0*sin(-angle_0);
    //parameter OpenIPSL.Types.PerUnit Iq0=(Isr0*sin(-angle_0) + cos(-angle_0)*Isi0);
    parameter Real CoB=M_b/S_b "Change of base";

    //parameter OpenIPSL.Types.PerUnit Itermr(start=ir0);
    //parameter OpenIPSL.Types.PerUnit Itermq(start=ii0);
    ///parameter OpenIPSL.Types.PerUnit Imaxd( start=0);
    //parameter OpenIPSL.Types.PerUnit Imaxq( start=0);
    //parameter OpenIPSL.Types.PerUnit Ireal(start=0);
    //parameter OpenIPSL.Types.PerUnit Iimag(start=0);
    parameter OpenIPSL.Types.PerUnit Ir(start=ir0);
    parameter OpenIPSL.Types.PerUnit Ii(start=ii0);
    parameter Real Imax=1.4;

    parameter OpenIPSL.Types.PerUnit Ed0 = Edcalc0;
    parameter OpenIPSL.Types.PerUnit Eq0 = Eqcalc0;
    parameter OpenIPSL.Types.PerUnit Eqcalc0 = vi0;
    parameter OpenIPSL.Types.PerUnit Edcalc0 = vr0;
    parameter OpenIPSL.Types.PerUnit Iqneg0 = (Eqcalc0 - Ip0*Xe)/Re;
    parameter OpenIPSL.Types.PerUnit Ip0 = (Edcalc0- v0 + Iqneg0*Xe)/Re;
    parameter OpenIPSL.Types.PerUnit Iq0 = Iqneg0;



  equation
    anglev = atan2(p.vi, p.vr);
    VT = sqrt(p.vr*p.vr + p.vi*p.vi);
    IT = sqrt(p.ii^2 + p.ir^2);
    delta = anglev;
    Angle_t = delta;



   annotation (Icon(coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},
              {140,140}}), graphics={
          Rectangle(extent={{-140,140},{140,-140}}, lineColor={28,108,200}),
          Text(
            extent={{-126,90},{-66,50}},
            textColor={0,0,255},
            textString="IQCMD"),
          Text(
            extent={{-126,-50},{-66,-90}},
            textColor={0,0,255},
            textString="IPCMD"),
          Text(
            extent={{-136,-100},{-104,-140}},
            textColor={0,0,255},
            textString="IQ0"),
          Text(
            extent={{-76,-100},{-44,-140}},
            textColor={0,0,255},
            textString="IP0"),
          Text(
            extent={{-24,-106},{4,-138}},
            textColor={0,0,255},
            textString="V0"),
          Text(
            extent={{46,-104},{74,-136}},
            textColor={0,0,255},
            textString="Q0"),
          Text(
            extent={{106,-104},{134,-136}},
            textColor={0,0,255},
            textString="P0"),
          Text(
            extent={{-124,134},{-84,104}},
            textColor={0,0,255},
            textString="VT"),
          Text(
            extent={{44,132},{84,110}},
            textColor={0,0,255},
            textString="PGEN"),
          Text(
            extent={{96,128},{136,102}},
            textColor={0,0,255},
            textString="QGEN"),
          Text(
            extent={{-72,134},{-32,104}},
            textColor={0,0,255},
            textString="IT"),
          Text(
            extent={{-12,136},{28,106}},
            textColor={0,0,255},
            textString="Angle"),
          Text(
            extent={{18,-110},{38,-138}},
            textColor={0,0,255},
            textString="I0")}),                                   Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-140,-140},{140,140}})));
  end baseREGC_B;

  model LVACM
    //Modelica.Blocks.Interfaces.RealInput Ivpnt0
    //Modelica.Blocks.Interfaces.RealInput Ivpnt1
          parameter Real lvpnt0;
          parameter Real lvpnt1;
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{100,-20},{140,20}}),
          iconTransformation(extent={{100,-10},{120,10}})));
    Modelica.Blocks.Interfaces.RealInput Vt
      annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
          iconTransformation(extent={{-120,-10},{-100,10}})));

  equation
    y = smooth(1,noEvent(if Vt <= lvpnt0 then 0 elseif Vt >= lvpnt1 then 1 else (1/(lvpnt1-lvpnt0))*(Vt-lvpnt0)));
      annotation (Placement(transformation(extent={{-100,-20},{-60,20}}),
          iconTransformation(extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-28,-102})),
                  Placement(transformation(extent={{-100,-80},{-60,-40}}),
          iconTransformation(extent={{-20,-20},{20,20}},
          rotation=90,
          origin={66,-102})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),
                           graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0}),
          Text(
            extent={{-80,40},{80,-40}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="%name"),
          Line(points={{-32,66}}, color={28,108,200})}),           Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}})));
  end LVACM;

  model LVPL
    Modelica.Blocks.Interfaces.RealOutput y
      annotation (Placement(transformation(extent={{100,-20},{140,20}}),
          iconTransformation(extent={{100,-10},{120,10}})));
    Modelica.Blocks.Interfaces.RealInput V
      annotation (Placement(transformation(extent={{-120,-10},{-100,10}}),
          iconTransformation(extent={{-120,-10},{-100,10}})));
    //Modelica.Blocks.Interfaces.RealInput Brkpt
   // Modelica.Blocks.Interfaces.RealInput Zerox
    //Modelica.Blocks.Interfaces.RealInput Lvpl1
          parameter Real Brkpt;
          parameter Real Lvpl1;
          parameter Real Zerox;
  equation

    y = noEvent(if V < Zerox then 0 else if V > Brkpt then Lvpl1 else (V-Zerox)*(Lvpl1/(Brkpt-Zerox)));

    //y = smooth(1,noEvent(if V < Zerox then 0 else if V > Brkpt then 10e6 else (V-Zerox)*(Lvpl1/(Brkpt-Zerox))));

    //y = if ((ipcmd/(V-Zerox)) <= (Lvpl1/(Brkpt-Zerox)) and ipcmd > Lvpl1) then Lvpl1;

      annotation (Placement(transformation(extent={{-140,0},{-100,40}})),
                  Placement(transformation(extent={{-140,-40},{-100,0}})),
                  Placement(transformation(extent={{-140,-80},{-100,-40}})),
                Icon(coordinateSystem(preserveAspectRatio=false, extent={{-100,
              -100},{100,100}}),                                  graphics={
          Rectangle(
            extent={{-100,100},{100,-100}},
            lineColor={0,0,0}),
          Text(
            extent={{-80,40},{80,-40}},
            lineColor={28,108,200},
            textStyle={TextStyle.Bold},
            textString="%name")}),                                 Diagram(
          coordinateSystem(preserveAspectRatio=false, extent={{-100,-100},{
              100,100}})));
  end LVPL;

  model Thevenin

    Modelica.Blocks.Interfaces.RealInput Iqneg
    annotation (Placement(transformation(extent={{-140,42},{-100,82}}), iconTransformation(extent={{-140,48},{-100,88}})));
    Modelica.Blocks.Interfaces.RealInput Ip
    annotation (Placement(transformation(extent={{-140,-20},{-100,20}}), iconTransformation(extent={{-140,-20},{-100,20}})));
    Modelica.Blocks.Interfaces.RealInput V
    annotation (Placement(transformation(extent={{-140,-86},{-100,-46}}), iconTransformation(extent={{-140,-92},{-100,-52}})));
    Modelica.Blocks.Interfaces.RealOutput Eqcalc
     annotation (Placement(transformation(extent={{100,26},{136,62}}), iconTransformation(extent={{100,28},{136,64}})));
    Modelica.Blocks.Interfaces.RealOutput Edcalc
     annotation (Placement(transformation(extent={{100,-50},{136,-14}}), iconTransformation(extent={{100,-54},{136,-18}})));

    parameter Types.PerUnit Re=0 "Generator Effective Resistance(0-0.01 pu)";
    parameter Types.PerUnit Xe=0.5 "Generator Effective Reactance(0.05-0.2 pu)";
  equation
   Eqcalc = 0 + (Iqneg*Re) + (Ip*Xe);
   Edcalc = V + (Ip*Re) - (Iqneg*Xe);
    annotation (Icon(graphics={Rectangle(extent={{-100,100},{100,-100}}, lineColor={28,108,200}), Text(
            extent={{-68,38},{66,-30}},
            textColor={28,108,200},
            textString="%name",
            textStyle={TextStyle.Bold})}));
  end Thevenin;

end BaseClasses;
