within OpenIPSL.Tests.Renewable.PSSE.PV;
model PV_plant_NewREGCB
  "PV source in a SMIB system to test functionality of REPCA plant controller model"
  extends BaseClasses.SMIBRenewable(SysData(fn=60), freq(k=SysData.fn),
    pwFault(t2=3));
  Electrical.Renewables.PSSE.PV_New
                                pV_New(
    P_0=1500000,
    Q_0=-5665800,
    v_0=1,
    angle_0(displayUnit="deg") = 0.02574992,
    QFunctionality=4,
    PFunctionality=0,
     redeclare
      OpenIPSL.Electrical.Renewables.PSSE.RenewableGeneratorConverter.REGC_B
      RenewableGenerator(
      rflag=true,
      pqflag=true),
     redeclare
      OpenIPSL.Electrical.Renewables.PSSE.RenewableElectricalController.REECDU1
      RenewableController(
      pfflag=true,
      vflag=true,
      qflag=true,
      pqflag=true,
      pflag=false,
      vcmpflag=false),
    redeclare
      OpenIPSL.Electrical.Renewables.PSSE.RenewablePlantController.REPCA1
      PlantController(
      Rc=0,
      Xc=0,
      Vref=0,
      vcflag=false,
      refflag=false,
      fflag=false))
    annotation (Placement(transformation(extent={{-60,-10},{-40,10}})));
equation
  connect(pV_New.pwPin, GEN1.p) annotation (Line(points={{-41.6667,0.909091},{
          -36,0.909091},{-36,0},{-30,0}},
                                      color={0,0,255}));
  connect(pwCurrent.ir, pV_New.branch_ir) annotation (Line(points={{-17.6,-6.6},
          {-17.6,-16},{-45,-16},{-45,-6.90909}}, color={0,0,127}));
  connect(pwCurrent.ii, pV_New.branch_ii) annotation (Line(points={{-14,-6.6},{-14,
          -20},{-55,-20},{-55,-6.90909}}, color={0,0,127}));
  connect(freq.y, pV_New.FREQ) annotation (Line(points={{-75,0},{-66,0},{-66,
          0.909091},{-58.8333,0.909091}},
                                color={0,0,127}));
  connect(pwVoltage.vr, pV_New.regulate_vr) annotation (Line(points={{-36.6,33.6},
          {-55,33.6},{-55,8.72727}}, color={0,0,127}));
  connect(pwVoltage.vi, pV_New.regulate_vi) annotation (Line(points={{-36.6,30},
          {-45,30},{-45,8.72727}}, color={0,0,127}));
  annotation (experiment(
      StopTime=5,
      __Dymola_NumberOfIntervals=50004,
      __Dymola_Algorithm="Dassl"), Documentation(info="<html>
<p>
Simulate for 5 seconds.
</p>
<p>This test system aims to show how to use the renewable PV plant component in all the possible control options. The model was developed following the
modeling template from the WECC PV Power Plant Dynamic Modeling Guide and PSSE manuals.</p>
<p>There are a total of eight reactive power control options (Constant local PF control, Constant local Q control, Local V control, Local coordinated
 V/Q control, Plant level Q control, Plant level V control, Plant level Q control + local coordinated V/Q control, Plant level V control + local
coordinated V/Q control), 2 active power options (No governor response, Governor response with up and down regulation).</p>
<p>Variables of interest:</p>
<ul>
<li><code>pV.RenewableGenerator.Pgen</code></li>
<li><code>pV.RenewableGenerator.Qgen</code></li>
<li><code>GEN1.v</code></li>
<li><code>FAULT.v</code></li>
<li><code>GEN2.v</code></li>
</ul>
</html>"),
    Diagram(coordinateSystem(extent={{-100,-80},{100,100}})),
    Icon(coordinateSystem(extent={{-100,-80},{100,100}})));
end PV_plant_NewREGCB;
