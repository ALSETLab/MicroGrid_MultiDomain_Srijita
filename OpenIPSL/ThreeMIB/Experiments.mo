within OpenIPSL.ThreeMIB;
package Experiments
 extends Modelica.Icons.ExamplesPackage;
 model ThreeMIB
   extends Modelica.Icons.Example;
   extends BaseModels.BaseNetwork.ThreeMIBPartial(
                                              infiniteBus(displayPF=true));
   BaseModels.GenerationUnits.Generator1 generator1_1(displayPF=true) annotation (Placement(transformation(extent={{-128,34},{-108,54}})));
   BaseModels.GenerationUnits.Generator2 generator2_1(displayPF=true) annotation (Placement(transformation(extent={{-128,-10},{-108,10}})));
   BaseModels.GenerationUnits.Generator3 generator3_1(displayPF=true) annotation (Placement(transformation(extent={{-126,-54},{-106,-34}})));
 equation
   connect(generator2_1.pwPin, B02.p) annotation (Line(points={{-107,0},{-86,0}}, color={0,0,255}));
   connect(generator3_1.pwPin, B03.p) annotation (Line(points={{-105,-44},{-86,-44}}, color={0,0,255}));
   connect(B01.p, generator1_1.pwPin) annotation (Line(points={{-86,44},{-107,44}}, color={0,0,255}));
   annotation (Diagram(coordinateSystem(extent={{-160,-100},{100,100}})), Icon(coordinateSystem(extent={{-160,-100},{100,100}})));
 end ThreeMIB;

 model ThreeMIB2
   extends BaseModels.BaseNetwork.ThreeMIBPartial2(
     B01(
       V_b=18000,    v_0=1.04,
       angle_0=-0.11510577316597),
     B02(
       V_b=18000,  v_0=1.04,
       angle_0=-0.11510577316597),
     B03(
       V_b=18000,  v_0=1.02,
       angle_0=-0.64785487370016),
     B04(
       V_b=500000,  v_0=1.016914,
       angle_0=-0.20030463146731),
     B05(
       V_b=500000,  v_0=0.9727782,
       angle_0=-0.73827933504843),
     B06(
       V_b=500000,  v_0=1,
       angle_0=-0.99776633612161),
     load(
       V_b=500000,
       P_0=1400000000,
       Q_0=100000000,
          v_0=1.016914,
       angle_0=-0.20030463146731),
     load1(
       V_b=500000,
       P_0=2000000000,
       Q_0=100000000,
           v_0=0.9727782,
       angle_0=-0.73827933504843),
     load2(
       V_b=500000,
       P_0=10000000000,
       Q_0=2000000000,
           v_0=1,
       angle_0=-0.99776633612161),
     IB(
       V_b=500000,
       P_0=9792000000,
       Q_0=2049824000,
       v_0=1,
       angle_0=-0.99776633612161,
       M_b=100000000000),
     twoWindingTransformer(
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     twoWindingTransformer1(
       VNOM1=18000,
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     twoWindingTransformer2(
       VNOM1=18000,
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     line1(R=0));
   BaseModels.GenerationUnits.Generator1 generator1_1(
     V_b=18000,
     P_0=1404000000,
     Q_0=434412700,
     v_0=1.04,
     angle_0=-0.11510577316597,                       displayPF=true) annotation (Placement(transformation(extent={{-122,34},{-102,54}})));
   BaseModels.GenerationUnits.Generator2 generator2_1(
     V_b=18000,
     P_0=1404000000,
     Q_0=434412700,
     v_0=1.04,
     angle_0=-0.11510577316597,                       displayPF=true) annotation (Placement(transformation(extent={{-124,-10},{-104,10}})));
    BaseModels.GenerationUnits.Generator3 generator3_1(
      V_b=18000,
      P_0=800000000,
      Q_0=466250100,
     v_0=1.02,
      angle_0=-0.64785487370016,
      displayPF=true) annotation (Placement(transformation(extent={{-122,-54},{-102,-34}})));
 equation
   connect(generator2_1.pwPin, B02.p) annotation (Line(points={{-103,0},{-86,0}}, color={0,0,255}));
   connect(generator1_1.pwPin, B01.p) annotation (Line(points={{-101,44},{-86,44}}, color={0,0,255}));
    connect(generator3_1.pwPin, B03.p) annotation (Line(points={{-101,-44},{-86,-44}}, color={0,0,255}));
    annotation (Diagram(coordinateSystem(extent={{-140,-100},{60,100}}), graphics={Rectangle(
            extent={{-140,100},{60,-100}},
            lineColor={28,108,200},
            lineThickness=0.5)}),                                          Icon(coordinateSystem(extent={{-140,-100},{60,100}})));
 end ThreeMIB2;

 model ThreeMIBEx
   extends BaseModels.BaseNetwork.ThreeMIBPartial2(
     B01(
       V_b=18000,    v_0=1.04,
       angle_0=-0.11510577316597),
     B02(
       V_b=18000,  v_0=1.04,
       angle_0=-0.11510577316597),
     B03(
       V_b=18000,  v_0=1.02,
       angle_0=-0.64785487370016),
     B04(
       V_b=500000,  v_0=1.016914,
       angle_0=-0.20030463146731),
     B05(
       V_b=500000,  v_0=0.9727782,
       angle_0=-0.73827933504843),
     B06(
       V_b=500000,  v_0=1,
       angle_0=-0.99776633612161),
     load(
       V_b=500000,
       P_0=1400000000,
       Q_0=100000000,
          v_0=1.016914,
       angle_0=-0.20030463146731),
     load1(
       V_b=500000,
       P_0=2000000000,
       Q_0=100000000,
           v_0=0.9727782,
       angle_0=-0.73827933504843),
     load2(
       V_b=500000,
       P_0=10000000000,
       Q_0=2000000000,
           v_0=1,
       angle_0=-0.99776633612161),
     IB(
       V_b=500000,
       P_0=9792000000,
       Q_0=2049824000,
       v_0=1,
       angle_0=-0.99776633612161,
       M_b=100000000000),
     twoWindingTransformer(
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     twoWindingTransformer1(
       VNOM1=18000,
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     twoWindingTransformer2(
       VNOM1=18000,
       VB1=18000,
       VNOM2=500000,
       VB2=500000),
     line1(R=0));
   BaseModels.GenerationUnitsEX.Generator1
                                         generator1_1(
     V_b=18000,
     P_0=1404000000,
     Q_0=434412700,
     v_0=1.04,
     angle_0=-0.11510577316597,                       displayPF=true) annotation (Placement(transformation(extent={{-122,34},{-102,54}})));
   BaseModels.GenerationUnitsEX.Generator2
                                         generator2_1(
     V_b=18000,
     P_0=1404000000,
     Q_0=434412700,
     v_0=1.04,
     angle_0=-0.11510577316597,                       displayPF=true) annotation (Placement(transformation(extent={{-124,-10},{-104,10}})));
  Electrical.Renewables.PSSE.PV_Srijita
                                 pV_Srijita(
      P_0=800000000,
      Q_0=466250100,
      v_0=1.02,
      angle_0(displayUnit="deg") = -0.64785487370016,
      displayPF=true,
     QFunctionality=0,
     PFunctionality=0,
     redeclare
       OpenIPSL.Electrical.Renewables.PSSE.RenewableGeneratorConverter.REGC_A
       RenewableGenerator(Lvplsw=true),
     redeclare
       OpenIPSL.Electrical.Renewables.PSSE.RenewableElectricalController.REECDU1
       baseREECD(
       pfflag=false,
       vflag=false,
       qflag=false,
       pqflag=false,
       pflag=false,
       vcmpflag=false,
       vref0=1),
     redeclare
       OpenIPSL.Electrical.Renewables.PSSE.RenewablePlantController.REPCA1
       PlantController(
       Rc=0,
       Xc=0,
       Vref=0,
       vcflag=false,
       refflag=false,
       fflag=false))
        annotation (Placement(transformation(extent={{-124,-54},{-104,-34}})));
   Electrical.Events.PwFault          pwFault(
      t2=2.15,
     R=0.5,
     X=0.5,
      t1=2)  annotation (Placement(transformation(extent={{-6,-84},{14,-64}})));
 equation
   connect(generator2_1.pwPin, B02.p) annotation (Line(points={{-103,0},{-86,0}}, color={0,0,255}));
   connect(generator1_1.pwPin, B01.p) annotation (Line(points={{-101,44},{-86,44}}, color={0,0,255}));
   connect(pV_Srijita.pwPin, B03.p) annotation (Line(points={{-104,-44},{-86,-44}}, color={0,0,255}));
   connect(pwFault.p, B05.p) annotation (Line(points={{-7.66667,-74},{-28,-74},{-28,-64},{-40,-64},{-40,-44},{-28,-44}}, color={0,0,255}));
 end ThreeMIBEx;

  model trial

    Modelica.ComplexBlocks.ComplexMath.RealToComplex rc annotation (Placement(transformation(extent={{-22,52},{-2,32}})));
    Modelica.Blocks.Sources.Constant const(k=2) annotation (Placement(transformation(extent={{-70,38},{-50,58}})));
    Modelica.Blocks.Sources.Constant const1(k=5) annotation (Placement(transformation(extent={{-72,4},{-52,24}})));
  equation



    connect(const.y, rc.im) annotation (Line(points={{-49,48},{-24,48}}, color={0,0,127}));
    connect(const1.y, rc.re) annotation (Line(points={{-51,14},{-32,14},{-32,36},{-24,36}}, color={0,0,127}));
  end trial;
end Experiments;
