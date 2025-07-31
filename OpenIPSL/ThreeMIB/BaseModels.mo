within OpenIPSL.ThreeMIB;
package BaseModels
  package GenerationUnits
    model InfiniteBus
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENCLS gENCLS(
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=100000000000,
        H=5,
        D=0,
        R_a=0,
        X_d=0.2) annotation (Placement(transformation(extent={{6,-20},{44,20}})));
    equation
      connect(gENCLS.p, pwPin) annotation (Line(points={{44,0},{110,0}}, color={0,0,255}));
    end InfiniteBus;

    model Generator3
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENROE gENROE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=890000000,
        Tpd0=5.3000,
        Tppd0=0.0480,
        Tppq0=0.0660,
        H=3.8590,
        D=0.0000,
        Xd=1.7200,
        Xq=1.6790,
        Xpd=0.4880,
        Xppd=0.3370,
        Xppq=0.3370,
        Xl=0.2660,
        S10=0.1300,
        S12=1.0670,
        R_a=0,
        Xpq=0.80000,
        Tpq0=0.6250)
                  annotation (Placement(transformation(extent={{-38,-26},{28,26}})));
    equation
      connect(gENROE.PMECH, gENROE.PMECH0) annotation (Line(points={{-44.6,15.6},{-74,15.6},{-74,52},{50,52},{50,13},{31.3,13}}, color={0,0,127}));
      connect(gENROE.p, pwPin) annotation (Line(points={{28,0},{110,0}}, color={0,0,255}));
      connect(gENROE.EFD0, gENROE.EFD) annotation (Line(points={{31.3,-13},{52,-13},{52,-40},{-70,-40},{-70,-15.6},{-44.6,-15.6}}, color={0,0,127}));
    end Generator3;

    model Generator1
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENSAE gENSAE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=1560000000,
        Tpd0=5.1000,
        Tppd0=0.0600,
        Tppq0=0.0940,
        H=4.5000,
        D=0.0000,
        Xd=0.8900,
        Xq=0.6600,
        Xpd=0.3600,
        Xppd=0.2900,
        Xppq=0.2900,
        Xl=0.2800,
        S10=0.0870,
        S12=0.2570,
        R_a=0.001900)                                        annotation (Placement(transformation(extent={{-36,-22},{34,28}})));
    equation
      connect(gENSAE.PMECH, gENSAE.PMECH0) annotation (Line(points={{-43,18},{-66,18},{-66,42},{56,42},{56,15.5},{37.5,15.5}}, color={0,0,127}));
      connect(gENSAE.EFD, gENSAE.EFD0) annotation (Line(points={{-43,-12},{-66,-12},{-66,-30},{56,-30},{56,-9.5},{37.5,-9.5}}, color={0,0,127}));
      connect(gENSAE.p, pwPin) annotation (Line(points={{34,3},{94,3},{94,0},{110,0}}, color={0,0,255}));
    end Generator1;

    model Generator2
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENSAE gENSAE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=1560000000,
        Tpd0=5.1000,
        Tppd0=0.0600,
        Tppq0=0.0940,
        H=4.5000,
        D=0.0000,
        Xd=0.8900,
        Xq=0.6600,
        Xpd=0.3600,
        Xppd=0.2900,
        Xppq=0.2900,
        Xl=0.2800,
        S10=0.0870,
        S12=0.2570,
        R_a=0.001900)                                        annotation (Placement(transformation(extent={{-36,-24},{32,28}})));
    equation
      connect(gENSAE.PMECH, gENSAE.PMECH0) annotation (Line(points={{-42.8,17.6},{-64,17.6},{-64,40},{50,40},{50,15},{35.4,15}}, color={0,0,127}));
      connect(gENSAE.EFD, gENSAE.EFD0) annotation (Line(points={{-42.8,-13.6},{-64,-13.6},{-64,-40},{54,-40},{54,-11},{35.4,-11}}, color={0,0,127}));
      connect(gENSAE.p, pwPin) annotation (Line(points={{32,2},{96,2},{96,0},{110,0}}, color={0,0,255}));
    end Generator2;
  end GenerationUnits;

  package BaseNetwork
    partial model ThreeMIBPartial

      Electrical.Buses.Bus B01(displayPF=true) annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
      Electrical.Buses.Bus B03(displayPF=true) annotation (Placement(transformation(extent={{-96,-54},{-76,-34}})));
      Electrical.Buses.Bus B04(displayPF=true) annotation (Placement(transformation(extent={{-40,34},{-20,54}})));
      Electrical.Buses.Bus B02(displayPF=true) annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
      Electrical.Branches.PwLine line1(
        R=0.0010,
        X=0.036000,
        G=0.0000,
        B=0.0000) annotation (Placement(transformation(extent={{-10,-54},{10,-34}})));
      Electrical.Loads.PSSE.Load load(P_0=1400000000, Q_0=100000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={4,41})));
      inner Electrical.SystemBase SysData annotation (Placement(transformation(extent={{-134,68},{-78,88}})));
      GenerationUnits.InfiniteBus infiniteBus annotation (Placement(transformation(extent={{78,-54},{58,-34}})));
      Electrical.Buses.Bus B05(displayPF=true) annotation (Placement(transformation(extent={{-38,-54},{-18,-34}})));
      Electrical.Buses.Bus B06(displayPF=true) annotation (Placement(transformation(extent={{20,-54},{40,-34}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer(
        CZ=1,
        R=0.000000,
        X=0.006410,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=500000) annotation (Placement(transformation(extent={{-68,34},{-48,54}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer1(
        CZ=1,
        R=0.000000,
        X=0.006410,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=500000) annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer2(
        CZ=1,
        R=0.000000,
        X=0.011200,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=18000) annotation (Placement(transformation(extent={{-70,-54},{-50,-34}})));
      Electrical.Loads.PSSE.Load load1(P_0=2000000000, Q_0=100000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={-40,-75})));
      Electrical.Loads.PSSE.Load load2(P_0=10000000000, Q_0=2000000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={48,-75})));
    equation
      connect(B01.p, twoWindingTransformer.p) annotation (Line(points={{-86,44},{-69,44}}, color={0,0,255}));
      connect(twoWindingTransformer.n, B04.p) annotation (Line(points={{-47,44},{-30,44}}, color={0,0,255}));
      connect(B02.p, twoWindingTransformer1.p) annotation (Line(points={{-86,0},{-69,0}}, color={0,0,255}));
      connect(twoWindingTransformer1.n, B04.p) annotation (Line(points={{-47,0},{-38,0},{-38,44},{-30,44}}, color={0,0,255}));
      connect(B03.p, twoWindingTransformer2.p) annotation (Line(points={{-86,-44},{-71,-44}}, color={0,0,255}));
      connect(twoWindingTransformer2.n, B05.p) annotation (Line(points={{-49,-44},{-28,-44}}, color={0,0,255}));
      connect(B05.p, line1.p) annotation (Line(points={{-28,-44},{-9,-44}}, color={0,0,255}));
      connect(load.p, B04.p) annotation (Line(points={{4,47},{4,66},{-40,66},{-40,44},{-30,44}}, color={0,0,255}));
      connect(B06.p, infiniteBus.pwPin) annotation (Line(points={{30,-44},{57,-44}}, color={0,0,255}));
      connect(line1.n, B06.p) annotation (Line(points={{9,-44},{30,-44}}, color={0,0,255}));
      connect(load2.p, infiniteBus.pwPin) annotation (Line(points={{48,-69},{48,-44},{57,-44}}, color={0,0,255}));
      connect(load1.p, B05.p) annotation (Line(points={{-40,-69},{-40,-44},{-28,-44}}, color={0,0,255}));
      connect(B04.p, line1.p) annotation (Line(points={{-30,44},{-20,44},{-20,-44},{-9,-44}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,100}})), Icon(coordinateSystem(extent={{-140,-100},{100,100}})));
    end ThreeMIBPartial;

    partial model ThreeMIBPartial2

      Electrical.Buses.Bus B01(displayPF=true) annotation (Placement(transformation(extent={{-96,34},{-76,54}})));
      Electrical.Buses.Bus B03(displayPF=true) annotation (Placement(transformation(extent={{-96,-54},{-76,-34}})));
      Electrical.Buses.Bus B04(displayPF=true) annotation (Placement(transformation(extent={{-40,34},{-20,54}})));
      Electrical.Buses.Bus B02(displayPF=true) annotation (Placement(transformation(extent={{-96,-10},{-76,10}})));
      Electrical.Branches.PwLine line1(
        R=0.0010,
        X=0.12,
        G=0.0000,
        B=0.0000) annotation (Placement(transformation(extent={{-10,-54},{10,-34}})));
      Electrical.Loads.PSSE.Load load(P_0=1400000000, Q_0=100000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={4,41})));
      inner Electrical.SystemBase SysData annotation (Placement(transformation(extent={{-134,68},{-78,88}})));
      Electrical.Machines.PSSE.GENCLS IB annotation (Placement(transformation(extent={{52,-54},{46,-34}})));
      Electrical.Buses.Bus B05(displayPF=true) annotation (Placement(transformation(extent={{-38,-54},{-18,-34}})));
      Electrical.Buses.Bus B06(displayPF=true) annotation (Placement(transformation(extent={{20,-54},{40,-34}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer(
        CZ=1,
        R=0.000000,
        X=0.006410,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=500000) annotation (Placement(transformation(extent={{-68,34},{-48,54}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer1(
        CZ=1,
        R=0.000000,
        X=0.006410,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=500000) annotation (Placement(transformation(extent={{-68,-10},{-48,10}})));
      Electrical.Branches.PSSE.TwoWindingTransformer twoWindingTransformer2(
        CZ=1,
        R=0.000000,
        X=0.011200,
        G=0.000000,
        B=0.000000,
        CW=1,
        VNOM1=18000,
        VNOM2=18000) annotation (Placement(transformation(extent={{-70,-54},{-50,-34}})));
      Electrical.Loads.PSSE.Load load1(P_0=2000000000, Q_0=100000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={-40,-75})));
      Electrical.Loads.PSSE.Load load2(P_0=10000000000, Q_0=2000000000)
                                      annotation (Placement(transformation(extent={{-7,-6},{7,6}},
            rotation=0,
            origin={40,-75})));
      Electrical.Branches.PwLine line2(
        R=0,
        X=0.036,
        G=0,
        B=0) annotation (Placement(transformation(
            extent={{-10,-10},{10,10}},
            rotation=270,
            origin={-14,6})));
    equation
      connect(B01.p, twoWindingTransformer.p) annotation (Line(points={{-86,44},{-69,44}}, color={0,0,255}));
      connect(twoWindingTransformer.n, B04.p) annotation (Line(points={{-47,44},{-30,44}}, color={0,0,255}));
      connect(B02.p, twoWindingTransformer1.p) annotation (Line(points={{-86,0},{-69,0}}, color={0,0,255}));
      connect(twoWindingTransformer1.n, B04.p) annotation (Line(points={{-47,0},{-38,0},{-38,44},{-30,44}}, color={0,0,255}));
      connect(B03.p, twoWindingTransformer2.p) annotation (Line(points={{-86,-44},{-71,-44}}, color={0,0,255}));
      connect(twoWindingTransformer2.n, B05.p) annotation (Line(points={{-49,-44},{-28,-44}}, color={0,0,255}));
      connect(B05.p, line1.p) annotation (Line(points={{-28,-44},{-9,-44}}, color={0,0,255}));
      connect(load.p, B04.p) annotation (Line(points={{4,47},{4,66},{-40,66},{-40,44},{-30,44}}, color={0,0,255}));
      connect(line1.n, B06.p) annotation (Line(points={{9,-44},{30,-44}}, color={0,0,255}));
      connect(load1.p, B05.p) annotation (Line(points={{-40,-69},{-40,-44},{-28,-44}}, color={0,0,255}));
      connect(B06.p, IB.p) annotation (Line(points={{30,-44},{46,-44}}, color={0,0,255}));
      connect(load2.p, IB.p) annotation (Line(points={{40,-69},{40,-44},{46,-44}}, color={0,0,255}));
      connect(B04.p, line2.p) annotation (Line(points={{-30,44},{-14,44},{-14,15}}, color={0,0,255}));
      connect(line2.n, line1.p) annotation (Line(points={{-14,-3},{-14,-44},{-9,-44}}, color={0,0,255}));
      annotation (Diagram(coordinateSystem(extent={{-140,-100},{100,100}})), Icon(coordinateSystem(extent={{-140,-100},{100,100}})));
    end ThreeMIBPartial2;
  end BaseNetwork;

  package GenerationUnitsEX
    model InfiniteBus
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENCLS gENCLS(
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=100000000000,
        H=5,
        D=0,
        R_a=0,
        X_d=0.2) annotation (Placement(transformation(extent={{6,-20},{44,20}})));
    equation
      connect(gENCLS.p, pwPin) annotation (Line(points={{44,0},{110,0}}, color={0,0,255}));
    end InfiniteBus;

    model Generator3
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENROE gENROE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=890000000,
        Tpd0=5.3000,
        Tppd0=0.0480,
        Tppq0=0.0660,
        H=3.8590,
        D=0.0000,
        Xd=1.7200,
        Xq=1.6790,
        Xpd=0.4880,
        Xppd=0.3370,
        Xppq=0.3370,
        Xl=0.2660,
        S10=0.1300,
        S12=1.0670,
        R_a=0,
        Xpq=0.80000,
        Tpq0=0.6250)
                  annotation (Placement(transformation(extent={{-38,-26},{28,26}})));
      Electrical.Controls.PSSE.ES.SCRX sCRX(
        T_AT_B=1,
        T_B=1,
        K=150,
        T_E=0.05,
        E_MIN=-5,
        E_MAX=5,
        C_SWITCH=true,
        r_cr_fd=0) annotation (Placement(transformation(extent={{10,-70},{-20,-40}})));
      Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-70,-92},{-50,-72}})));
    equation
      connect(gENROE.PMECH, gENROE.PMECH0) annotation (Line(points={{-44.6,15.6},{-74,15.6},{-74,52},{50,52},{50,13},{31.3,13}}, color={0,0,127}));
      connect(gENROE.p, pwPin) annotation (Line(points={{28,0},{110,0}}, color={0,0,255}));
      connect(sCRX.EFD, gENROE.EFD) annotation (Line(points={{-21.5,-55},{-70,-55},{-70,-15.6},{-44.6,-15.6}}, color={0,0,127}));
      connect(const.y, sCRX.VOEL) annotation (Line(points={{-49,-82},{-5,-82},{-5,-71.5}}, color={0,0,127}));
      connect(sCRX.VUEL, sCRX.VOEL) annotation (Line(points={{1,-71.5},{1,-82},{-5,-82},{-5,-71.5}}, color={0,0,127}));
      connect(sCRX.VOTHSG, sCRX.VOEL) annotation (Line(points={{11.5,-49},{28,-49},{28,-82},{-5,-82},{-5,-71.5}}, color={0,0,127}));
      connect(gENROE.ETERM, sCRX.ECOMP) annotation (Line(points={{31.3,-7.8},{48,-7.8},{48,-55},{11.5,-55}}, color={0,0,127}));
      connect(gENROE.EFD0, sCRX.EFD0) annotation (Line(points={{31.3,-13},{58,-13},{58,-61},{11.5,-61}}, color={0,0,127}));
      connect(gENROE.XADIFD, sCRX.XADIFD) annotation (Line(points={{31.3,-23.4},{44,-23.4},{44,-78},{-17,-78},{-17,-71.5}}, color={0,0,127}));
    end Generator3;

    model Generator1
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENSAE gENSAE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=1560000000,
        Tpd0=5.1000,
        Tppd0=0.0600,
        Tppq0=0.0940,
        H=4.5000,
        D=0.0000,
        Xd=0.8900,
        Xq=0.6600,
        Xpd=0.3600,
        Xppd=0.2900,
        Xppq=0.2900,
        Xl=0.2800,
        S10=0.0870,
        S12=0.2570,
        R_a=0.001900)                                        annotation (Placement(transformation(extent={{-36,-22},{34,28}})));
      Electrical.Controls.PSSE.ES.SCRX sCRX(
        T_AT_B=1,
        T_B=1,
        K=100,
        T_E=0.05,
        E_MIN=-5,
        E_MAX=5,
        C_SWITCH=true,
        r_cr_fd=0) annotation (Placement(transformation(extent={{18,-70},{-12,-40}})));
      Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-60,-94},{-40,-74}})));
    equation
      connect(gENSAE.PMECH, gENSAE.PMECH0) annotation (Line(points={{-43,18},{-66,18},{-66,42},{56,42},{56,15.5},{37.5,15.5}}, color={0,0,127}));
      connect(gENSAE.p, pwPin) annotation (Line(points={{34,3},{94,3},{94,0},{110,0}}, color={0,0,255}));
      connect(sCRX.ECOMP, gENSAE.ETERM) annotation (Line(points={{19.5,-55},{19.5,-56},{50,-56},{50,-4.5},{37.5,-4.5}}, color={0,0,127}));
      connect(gENSAE.XADIFD, sCRX.XADIFD) annotation (Line(points={{37.5,-19.5},{37.5,-80},{-9,-80},{-9,-71.5}}, color={0,0,127}));
      connect(gENSAE.EFD, sCRX.EFD) annotation (Line(points={{-43,-12},{-64,-12},{-64,-55},{-13.5,-55}}, color={0,0,127}));
      connect(gENSAE.EFD0, sCRX.EFD0) annotation (Line(points={{37.5,-9.5},{62,-9.5},{62,-61},{19.5,-61}}, color={0,0,127}));
      connect(const.y, sCRX.VOEL) annotation (Line(points={{-39,-84},{3,-84},{3,-71.5}}, color={0,0,127}));
      connect(sCRX.VUEL, sCRX.VOEL) annotation (Line(points={{9,-71.5},{9,-84},{3,-84},{3,-71.5}}, color={0,0,127}));
      connect(sCRX.VOTHSG, sCRX.VOEL) annotation (Line(points={{19.5,-49},{28,-49},{28,-84},{3,-84},{3,-71.5}}, color={0,0,127}));
    end Generator1;

    model Generator2
      extends Interfaces.Generator;
      Electrical.Machines.PSSE.GENSAE gENSAE(
      V_b = 18000,
        P_0=P_0,
        Q_0=Q_0,
        v_0=v_0,
        angle_0=angle_0,
        M_b=1560000000,
        Tpd0=5.1000,
        Tppd0=0.0600,
        Tppq0=0.0940,
        H=4.5000,
        D=0.0000,
        Xd=0.8900,
        Xq=0.6600,
        Xpd=0.3600,
        Xppd=0.2900,
        Xppq=0.2900,
        Xl=0.2800,
        S10=0.0870,
        S12=0.2570,
        R_a=0.001900)                                        annotation (Placement(transformation(extent={{-36,-24},{32,28}})));
      Electrical.Controls.PSSE.ES.SCRX sCRX(
        T_AT_B=1,
        T_B=1,
        K=100,
        T_E=0.05,
        E_MIN=-5,
        E_MAX=5,
        C_SWITCH=true,
        r_cr_fd=0) annotation (Placement(transformation(extent={{18,-72},{-12,-42}})));
      Modelica.Blocks.Sources.Constant const(k=0) annotation (Placement(transformation(extent={{-66,-92},{-46,-72}})));
    equation
      connect(gENSAE.PMECH, gENSAE.PMECH0) annotation (Line(points={{-42.8,17.6},{-64,17.6},{-64,40},{50,40},{50,15},{35.4,15}}, color={0,0,127}));
      connect(gENSAE.p, pwPin) annotation (Line(points={{32,2},{96,2},{96,0},{110,0}}, color={0,0,255}));
      connect(gENSAE.EFD, sCRX.EFD) annotation (Line(points={{-42.8,-13.6},{-58,-13.6},{-58,-57},{-13.5,-57}}, color={0,0,127}));
      connect(const.y, sCRX.VOEL) annotation (Line(points={{-45,-82},{3,-82},{3,-73.5}}, color={0,0,127}));
      connect(sCRX.VUEL, sCRX.VOEL) annotation (Line(points={{9,-73.5},{9,-82},{3,-82},{3,-73.5}}, color={0,0,127}));
      connect(sCRX.VOTHSG, sCRX.VOEL) annotation (Line(points={{19.5,-51},{40,-51},{40,-82},{3,-82},{3,-73.5}}, color={0,0,127}));
      connect(sCRX.XADIFD, gENSAE.XADIFD) annotation (Line(points={{-9,-73.5},{-9,-88},{46,-88},{46,-21.4},{35.4,-21.4}}, color={0,0,127}));
      connect(gENSAE.ETERM, sCRX.ECOMP) annotation (Line(points={{35.4,-5.8},{60,-5.8},{60,-57},{19.5,-57}}, color={0,0,127}));
      connect(gENSAE.EFD0, sCRX.EFD0) annotation (Line(points={{35.4,-11},{56,-11},{56,-63},{19.5,-63}}, color={0,0,127}));
    end Generator2;
  end GenerationUnitsEX;
end BaseModels;
