within ;
package Project2_PriusDrivetrain
  model Ecvt
    Modelica.Mechanics.Rotational.Components.IdealPlanetary idealPlanetary(ratio=2.6)
      annotation (Placement(transformation(extent={{34,-6},{54,14}})));
    Modelica.Mechanics.Rotational.Components.Inertia sun(J=1)
      annotation (Placement(transformation(extent={{-36,-6},{-16,14}})));
    Modelica.Mechanics.Rotational.Components.Inertia ring(J=1, w(start=1))
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=0,
          origin={88,66})));
    Modelica.Mechanics.Rotational.Components.Inertia carrier(J=1)
      annotation (Placement(transformation(extent={{-28,70},{-8,90}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensorRing
      annotation (Placement(transformation(extent={{64,2},{84,22}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-56,-38})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensorCarrier
      annotation (Placement(transformation(extent={{-18,28},{2,48}})));
    Modelica.Mechanics.Rotational.Sensors.SpeedSensor speedSensor1
      annotation (Placement(transformation(extent={{-30,-72},{-10,-52}})));
    Modelica.Blocks.Interfaces.RealInput engine_torque
      annotation (Placement(transformation(extent={{-120,38},{-80,78}})));
    Modelica.Mechanics.Rotational.Interfaces.Flange_b flange_b
      annotation (Placement(transformation(extent={{90,4},{110,24}})));
    Modelica.Blocks.Interfaces.RealInput MG2_Torque annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={44,-100})));
    Modelica.Mechanics.Rotational.Sources.Torque torque2
      annotation (Placement(transformation(extent={{-62,46},{-42,66}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque1
      annotation (Placement(transformation(extent={{50,-68},{70,-48}})));
    Modelica.Blocks.Interfaces.RealInput engine_rpm annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=0,
          origin={-100,94})));
    Modelica.Mechanics.Rotational.Sources.Speed speed
      annotation (Placement(transformation(extent={{-64,84},{-44,104}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
      annotation (Placement(transformation(extent={{-4,-6},{16,14}})));
    Modelica.Blocks.Interfaces.RealOutput MG1_Torque
      "Connector of Real output signal" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-100,0})));
    Modelica.Blocks.Interfaces.RealOutput MG1_RPM
      "Connector of Real output signal" annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={-102,-38})));
  equation
    connect(idealPlanetary.ring,torqueSensorRing. flange_a)
      annotation (Line(points={{54,4},{62,4},{62,12},{64,12}},
                                                 color={0,0,0}));
    connect(carrier.flange_b,torqueSensorCarrier. flange_a)
      annotation (Line(points={{-8,80},{-2,80},{-2,52},{-22,52},{-22,38},{-18,
            38}},                              color={0,0,0}));
    connect(torqueSensorCarrier.flange_b,idealPlanetary. carrier) annotation (
        Line(points={{2,38},{28,38},{28,8},{34,8}},
                  color={0,0,0}));
    connect(speedSensor.flange,sun. flange_a) annotation (Line(points={{-46,-38},
            {-46,4},{-36,4}},                                 color={0,0,0}));
    connect(speedSensor1.flange,torqueSensorRing. flange_a) annotation (Line(
          points={{-30,-62},{-34,-62},{-34,-14},{58,-14},{58,4},{62,4},{62,8},{
            64,8},{64,12}},                             color={0,0,0}));
    connect(torque2.flange, carrier.flange_a)
      annotation (Line(points={{-42,56},{-34,56},{-34,80},{-28,80}},
                                                            color={0,0,0}));
    connect(engine_torque, torque2.tau) annotation (Line(points={{-100,58},{-70,
            58},{-70,56},{-64,56}}, color={0,0,127}));
    connect(MG2_Torque, torque1.tau) annotation (Line(points={{44,-100},{44,-64},
            {42,-64},{42,-58},{48,-58}}, color={0,0,127}));
    connect(torque1.flange, idealPlanetary.ring) annotation (Line(points={{70,
            -58},{74,-58},{74,-26},{60,-26},{60,4},{54,4}}, color={0,0,0}));
    connect(engine_rpm, speed.w_ref)
      annotation (Line(points={{-100,94},{-66,94}}, color={0,0,127}));
    connect(speed.flange, carrier.flange_a) annotation (Line(points={{-44,94},{
            -34,94},{-34,80},{-28,80}}, color={0,0,0}));
    connect(idealPlanetary.sun, torqueSensor.flange_b)
      annotation (Line(points={{34,4},{16,4}}, color={0,0,0}));
    connect(sun.flange_b, torqueSensor.flange_a)
      annotation (Line(points={{-16,4},{-4,4}}, color={0,0,0}));
    connect(torqueSensor.tau, MG1_Torque) annotation (Line(points={{-2,-7},{-10,
            -7},{-10,18},{-86,18},{-86,0},{-100,0}}, color={0,0,127}));
    connect(speedSensor.w, MG1_RPM)
      annotation (Line(points={{-67,-38},{-102,-38}}, color={0,0,127}));
    connect(flange_b, ring.flange_b) annotation (Line(points={{100,14},{88,14},
            {88,52},{102,52},{102,66},{98,66}}, color={0,0,0}));
    connect(ring.flange_a, torqueSensorRing.flange_b) annotation (Line(points={
            {78,66},{72,66},{72,26},{86,26},{86,12},{84,12}}, color={0,0,0}));
  end Ecvt;

  model modeselection_Tsplit
    Modelica.Blocks.Interfaces.RealInput cyclespeed
      "Connector of Real input signal" annotation (Placement(transformation(
            extent={{-112,48},{-88,72}}), iconTransformation(extent={{-112,48},
              {-88,72}})));
    Modelica.Blocks.Math.Gain wheelrpm(k=13.726)
      annotation (Placement(transformation(extent={{-80,50},{-60,70}})));
    Modelica.Blocks.Math.Product powerreq
      annotation (Placement(transformation(extent={{-8,-90},{12,-70}})));
    Modelica.Blocks.Math.Gain wheelrads(k=0.1047)
      annotation (Placement(transformation(extent={{-40,-68},{-20,-48}})));
    Modelica.Blocks.Logical.And starting
      annotation (Placement(transformation(extent={{40,72},{54,86}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold=300)
      annotation (Placement(transformation(extent={{-8,82},{4,94}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,70},{110,90}})));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold=
         200)
      annotation (Placement(transformation(extent={{2,8},{14,20}})));
    Modelica.Blocks.Logical.And deplete
      annotation (Placement(transformation(extent={{42,18},{56,32}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y1
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,16},{110,36}})));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold1(threshold=
         300)
      annotation (Placement(transformation(extent={{-2,32},{10,44}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y2
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,-42},{110,-22}})));
    Modelica.Blocks.Logical.And sustain
      annotation (Placement(transformation(extent={{48,-40},{62,-26}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold2(threshold=
          200)
      annotation (Placement(transformation(extent={{2,-20},{14,-8}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y3
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,-74},{110,-54}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold
      annotation (Placement(transformation(extent={{62,-70},{76,-56}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold
      annotation (Placement(transformation(extent={{-26,62},{-16,72}})));
    Modelica.Blocks.Logical.And sustain1
      annotation (Placement(transformation(extent={{20,-54},{34,-40}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1
      annotation (Placement(transformation(extent={{-2,-60},{8,-50}})));
    Modelica.Blocks.Interfaces.RealInput torque_req_
      "Connector of Real input signal" annotation (Placement(transformation(
            extent={{-112,-14},{-88,10}}),iconTransformation(extent={{-112,48},
              {-88,72}})));
    Modelica.Blocks.Interfaces.RealOutput Power_Req
      annotation (Placement(transformation(extent={{90,-104},{110,-84}})));
  equation
    connect(cyclespeed, wheelrpm.u)
      annotation (Line(points={{-100,60},{-82,60}}, color={0,0,127}));
    connect(wheelrads.y, powerreq.u1) annotation (Line(points={{-19,-58},{-12,
            -58},{-12,-68},{-16,-68},{-16,-74},{-10,-74}}, color={0,0,127}));
    connect(wheelrpm.y, wheelrads.u) annotation (Line(points={{-59,60},{-50,60},
            {-50,-58},{-42,-58}}, color={0,0,127}));
    connect(starting.u1, lessEqualThreshold.y) annotation (Line(points={{38.6,
            79},{16,79},{16,88},{4.6,88}}, color={255,0,255}));
    connect(lessEqualThreshold.u, wheelrpm.y) annotation (Line(points={{-9.2,88},
            {-52,88},{-52,66},{-54,66},{-54,60},{-59,60}}, color={0,0,127}));
    connect(starting.y, y)
      annotation (Line(points={{54.7,79},{100,80}}, color={255,0,255}));
    connect(deplete.y, y1)
      annotation (Line(points={{56.7,25},{100,26}}, color={255,0,255}));
    connect(greaterEqualThreshold1.y, deplete.u1) annotation (Line(points={{
            10.6,38},{32,38},{32,25},{40.6,25}}, color={255,0,255}));
    connect(wheelrpm.y, greaterEqualThreshold1.u) annotation (Line(points={{-59,
            60},{-50,60},{-50,38},{-3.2,38}}, color={0,0,127}));
    connect(sustain.y, y2)
      annotation (Line(points={{62.7,-33},{100,-32}}, color={255,0,255}));
    connect(sustain.u1, greaterEqualThreshold1.y) annotation (Line(points={{
            46.6,-33},{34,-33},{34,24},{32,24},{32,38},{10.6,38}}, color={255,0,
            255}));
    connect(lessThreshold.y, y3) annotation (Line(points={{76.7,-63},{86,-63},{
            86,-64},{100,-64}}, color={255,0,255}));
    connect(greaterEqualThreshold.y, deplete.u2) annotation (Line(points={{14.6,
            14},{40.6,14},{40.6,19.4}}, color={255,0,255}));
    connect(sustain1.y, sustain.u2) annotation (Line(points={{34.7,-47},{46.6,
            -47},{46.6,-38.6}}, color={255,0,255}));
    connect(lessEqualThreshold2.y, sustain1.u1) annotation (Line(points={{14.6,
            -14},{22,-14},{22,-36},{12,-36},{12,-47},{18.6,-47}}, color={255,0,
            255}));
    connect(greaterThreshold1.u, lessEqualThreshold2.u) annotation (Line(points=
           {{-3,-55},{-8,-55},{-8,-14},{0.8,-14}}, color={0,0,127}));
    connect(sustain1.u2, greaterThreshold1.y) annotation (Line(points={{18.6,
            -52.6},{12,-52.6},{12,-55},{8.5,-55}}, color={255,0,255}));
    connect(greaterThreshold.y, starting.u2) annotation (Line(points={{-15.5,67},
            {38.6,67},{38.6,73.4}}, color={255,0,255}));
    connect(greaterThreshold.u, greaterEqualThreshold.u)
      annotation (Line(points={{-27,67},{-27,14},{0.8,14}}, color={0,0,127}));
    connect(lessThreshold.u, powerreq.u2) annotation (Line(points={{60.6,-63},{
            28,-63},{28,-86},{-10,-86}}, color={0,0,127}));
    connect(torque_req_, greaterEqualThreshold.u) annotation (Line(points={{
            -100,-2},{-64,-2},{-64,14},{0.8,14}}, color={0,0,127}));
    connect(torque_req_, powerreq.u2) annotation (Line(points={{-100,-2},{-64,
            -2},{-64,-86},{-10,-86}}, color={0,0,127}));
    connect(torque_req_, lessEqualThreshold2.u) annotation (Line(points={{-100,
            -2},{-64,-2},{-64,14},{-6,14},{-6,-14},{0.8,-14}}, color={0,0,127}));
    connect(powerreq.y, Power_Req) annotation (Line(points={{13,-80},{84,-80},{
            84,-94},{100,-94}}, color={0,0,127}));
  end modeselection_Tsplit;

  block Add4 "Output the sum of the four inputs"
    extends Modelica.Blocks.Icons.Block;

    parameter Real k1=+1 "Gain of input signal 1";
    parameter Real k2=+1 "Gain of input signal 2";
    parameter Real k3=+1 "Gain of input signal 3";
    parameter Real k4=+1 "Gain of input signal 4";
    Modelica.Blocks.Interfaces.RealInput u1 "Connector of Real input signal 1" annotation (
        Placement(transformation(extent={{-140,60},{-100,100}})));
    Modelica.Blocks.Interfaces.RealInput u2 "Connector of Real input signal 2" annotation (
        Placement(transformation(extent={{-140,10},{-100,20}})));
    Modelica.Blocks.Interfaces.RealInput u3 "Connector of Real input signal 3" annotation (
        Placement(transformation(extent={{-140,-40},{-100,-60}})));
    Modelica.Blocks.Interfaces.RealInput u4 "Connector of Real input signal 3" annotation (
        Placement(transformation(extent={{-140,-100},{-100,-80}})));
    Modelica.Blocks.Interfaces.RealOutput y "Connector of Real output signal" annotation (
        Placement(transformation(extent={{100,-10},{120,10}})));

  equation
    y = k1*u1 + k2*u2 + k3*u3+ k4*u4;
    annotation (
      Documentation(info="<html>
<p>
This blocks computes output <strong>y</strong> as <em>sum</em> of the
three input signals <strong>u1</strong>, <strong>u2</strong> and <strong>u3</strong>:
</p>
<blockquote><pre>
<strong>y</strong> = k1*<strong>u1</strong> + k2*<strong>u2</strong> + k3*<strong>u3</strong>;
</pre></blockquote>
<p>
Example:
</p>
<blockquote><pre>
   parameter:   k1= +2, k2= -3, k3=1;

results in the following equations:

   y = 2 * u1 - 3 * u2 + u3;
</pre></blockquote>

</html>"),
      Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
              100}}), graphics={
          Text(
            extent={{-100,50},{5,90}},
            textString="%k1"),
          Text(
            extent={{-100,-20},{5,20}},
            textString="%k2"),
          Text(
            extent={{-100,-50},{5,-90}},
            textString="%k3"),
          Text(
            extent={{10,40},{90,-40}},
            textString="+")}));
  end Add4;

  model T_split
    Modelica.Blocks.Sources.TimeTable timeTable(table=[0,0; 1,0; 2,0; 3,0; 4,0; 5,
          0; 6,0; 7,0; 8,0; 9,0; 10,0; 11,0; 12,0; 13,0; 14,0; 15,0; 16,0; 17,0;
          18,0; 19,0; 20,0; 21,3; 22,5.9; 23,8.6; 24,11.5; 25,14.3; 26,16.9; 27,
          17.3; 28,18.1; 29,20.7; 30,21.7; 31,22.4; 32,22.5; 33,22.1; 34,21.5; 35,
          20.9; 36,20.4; 37,19.8; 38,17; 39,14.9; 40,14.9; 41,15.2; 42,15.5; 43,
          16; 44,17.1; 45,19.1; 46,21.1; 47,22.7; 48,22.9; 49,22.7; 50,22.6; 51,
          21.3; 52,19; 53,17.1; 54,15.8; 55,15.8; 56,17.7; 57,19.8; 58,21.6; 59,
          23.2; 60,24.2; 61,24.6; 62,24.9; 63,25; 64,24.6; 65,24.5; 66,24.7; 67,
          24.8; 68,24.7; 69,24.6; 70,24.6; 71,25.1; 72,25.6; 73,25.7; 74,25.4; 75,
          24.9; 76,25; 77,25.4; 78,26; 79,26; 80,25.7; 81,26.1; 82,26.7; 83,27.5;
          84,28.6; 85,29.3; 86,29.8; 87,30.1; 88,30.4; 89,30.7; 90,30.7; 91,30.5;
          92,30.4; 93,30.3; 94,30.4; 95,30.8; 96,30.4; 97,29.9; 98,29.5; 99,29.8;
          100,30.3; 101,30.7; 102,30.9; 103,31; 104,30.9; 105,30.4; 106,29.8; 107,
          29.9; 108,30.2; 109,30.7; 110,31.2; 111,31.8; 112,32.2; 113,32.4; 114,
          32.2; 115,31.7; 116,28.6; 117,25.3; 118,22; 119,18.7; 120,15.4; 121,
          12.1; 122,8.8; 123,5.5; 124,2.2; 125,0; 126,0; 127,0; 128,0; 129,0; 130,
          0; 131,0; 132,0; 133,0; 134,0; 135,0; 136,0; 137,0; 138,0; 139,0; 140,0;
          141,0; 142,0; 143,0; 144,0; 145,0; 146,0; 147,0; 148,0; 149,0; 150,0;
          151,0; 152,0; 153,0; 154,0; 155,0; 156,0; 157,0; 158,0; 159,0; 160,0;
          161,0; 162,0; 163,0; 164,3.3; 165,6.6; 166,9.9; 167,13.2; 168,16.5; 169,
          19.8; 170,22.2; 171,24.3; 172,25.8; 173,26.4; 174,25.7; 175,25.1; 176,
          24.7; 177,25; 178,25.2; 179,25.4; 180,25.8; 181,27.2; 182,26.5; 183,24;
          184,22.7; 185,19.4; 186,17.7; 187,17.2; 188,18.1; 189,18.6; 190,20; 191,
          22.2; 192,24.5; 193,27.3; 194,30.5; 195,33.5; 196,36.2; 197,37.3; 198,
          39.3; 199,40.5; 200,42.1; 201,43.5; 202,45.1; 203,46; 204,46.8; 205,
          47.5; 206,47.5; 207,47.3; 208,47.2; 209,47; 210,47; 211,47; 212,47; 213,
          47; 214,47.2; 215,47.4; 216,47.9; 217,48.5; 218,49.1; 219,49.5; 220,50;
          221,50.6; 222,51; 223,51.5; 224,52.2; 225,53.2; 226,54.1; 227,54.6; 228,
          54.9; 229,55; 230,54.9; 231,54.6; 232,54.6; 233,54.8; 234,55.1; 235,
          55.5; 236,55.7; 237,56.1; 238,56.3; 239,56.6; 240,56.7; 241,56.7; 242,
          56.5; 243,56.5; 244,56.5; 245,56.5; 246,56.5; 247,56.5; 248,56.4; 249,
          56.1; 250,55.8; 251,55.1; 252,54.6; 253,54.2; 254,54; 255,53.7; 256,
          53.6; 257,53.9; 258,54; 259,54.1; 260,54.1; 261,53.8; 262,53.4; 263,53;
          264,52.6; 265,52.1; 266,52.4; 267,52; 268,51.9; 269,51.7; 270,51.5; 271,
          51.6; 272,51.8; 273,52.1; 274,52.5; 275,53; 276,53.5; 277,54; 278,54.9;
          279,55.4; 280,55.6; 281,56; 282,56; 283,55.8; 284,55.2; 285,54.5; 286,
          53.6; 287,52.5; 288,51.5; 289,51.5; 290,51.5; 291,51.1; 292,50.1; 293,
          50; 294,50.1; 295,50; 296,49.6; 297,49.5; 298,49.5; 299,49.5; 300,49.1;
          301,48.6; 302,48.1; 303,47.2; 304,46.1; 305,45; 306,43.8; 307,42.6; 308,
          41.5; 309,40.3; 310,38.5; 311,37; 312,35.2; 313,33.8; 314,32.5; 315,
          31.5; 316,30.6; 317,30.5; 318,30; 319,29; 320,27.5; 321,24.8; 322,21.5;
          323,20.1; 324,19.1; 325,18.5; 326,17; 327,15.5; 328,12.5; 329,10.8; 330,
          8; 331,4.7; 332,1.4; 333,0; 334,0; 335,0; 336,0; 337,0; 338,0; 339,0;
          340,0; 341,0; 342,0; 343,0; 344,0; 345,0; 346,0; 347,1; 348,4.3; 349,
          7.6; 350,10.9; 351,14.2; 352,17.3; 353,20; 354,22.5; 355,23.7; 356,25.2;
          357,26.6; 358,28.1; 359,30; 360,30.8; 361,31.6; 362,32.1; 363,32.8; 364,
          33.6; 365,34.5; 366,34.6; 367,34.9; 368,34.8; 369,34.5; 370,34.7; 371,
          35.5; 372,36; 373,36; 374,36; 375,36; 376,36; 377,36; 378,36.1; 379,
          36.4; 380,36.5; 381,36.4; 382,36; 383,35.1; 384,34.1; 385,33.5; 386,
          31.4; 387,29; 388,25.7; 389,23; 390,20.3; 391,17.5; 392,14.5; 393,12;
          394,8.7; 395,5.4; 396,2.1; 397,0; 398,0; 399,0; 400,0; 401,0; 402,0;
          403,2.6; 404,5.9; 405,9.2; 406,12.5; 407,15.8; 408,19.1; 409,22.4; 410,
          25; 411,25.6; 412,27.5; 413,29; 414,30; 415,30.1; 416,30; 417,29.7; 418,
          29.3; 419,28.8; 420,28; 421,25; 422,21.7; 423,18.4; 424,15.1; 425,11.8;
          426,8.5; 427,5.2; 428,1.9; 429,0; 430,0; 431,0; 432,0; 433,0; 434,0;
          435,0; 436,0; 437,0; 438,0; 439,0; 440,0; 441,0; 442,0; 443,0; 444,0;
          445,0; 446,0; 447,0; 448,3.3; 449,6.6; 450,9.9; 451,13.2; 452,16.5; 453,
          19.8; 454,23.1; 455,26.4; 456,27.8; 457,29.1; 458,31.5; 459,33; 460,
          33.6; 461,34.8; 462,35.1; 463,35.6; 464,36.1; 465,36; 466,36.1; 467,
          36.2; 468,36; 469,35.7; 470,36; 471,36; 472,35.6; 473,35.5; 474,35.4;
          475,35.2; 476,35.2; 477,35.2; 478,35.2; 479,35.2; 480,35.2; 481,35; 482,
          35.1; 483,35.2; 484,35.5; 485,35.2; 486,35; 487,35; 488,35; 489,34.8;
          490,34.6; 491,34.5; 492,33.5; 493,32; 494,30.1; 495,28; 496,25.5; 497,
          22.5; 498,19.8; 499,16.5; 500,13.2; 501,10.3; 502,7.2; 503,4; 504,1;
          505,0; 506,0; 507,0; 508,0; 509,0; 510,0; 511,1.2; 512,3.5; 513,5.5;
          514,6.5; 515,8.5; 516,9.6; 517,10.5; 518,11.9; 519,14; 520,16; 521,17.7;
          522,19; 523,20.1; 524,21; 525,22; 526,23; 527,23.8; 528,24.5; 529,24.9;
          530,25; 531,25; 532,25; 533,25; 534,25; 535,25; 536,25.6; 537,25.8; 538,
          26; 539,25.6; 540,25.2; 541,25; 542,25; 543,25; 544,24.4; 545,23.1; 546,
          19.8; 547,16.5; 548,13.2; 549,9.9; 550,6.6; 551,3.3; 552,0; 553,0; 554,
          0; 555,0; 556,0; 557,0; 558,0; 559,0; 560,0; 561,0; 562,0; 563,0; 564,0;
          565,0; 566,0; 567,0; 568,0; 569,3.3; 570,6.6; 571,9.9; 572,13; 573,14.6;
          574,16; 575,17; 576,17; 577,17; 578,17.5; 579,17.7; 580,17.7; 581,17.5;
          582,17; 583,16.9; 584,16.6; 585,17; 586,17.1; 587,17; 588,16.6; 589,
          16.5; 590,16.5; 591,16.6; 592,17; 593,17.6; 594,18.5; 595,19.2; 596,
          20.2; 597,21; 598,21.1; 599,21.2; 600,21.6; 601,22; 602,22.4; 603,22.5;
          604,22.5; 605,22.5; 606,22.7; 607,23.7; 608,25.1; 609,26; 610,26.5; 611,
          27; 612,26.1; 613,22.8; 614,19.5; 615,16.2; 616,12.9; 617,9.6; 618,6.3;
          619,3; 620,0; 621,0; 622,0; 623,0; 624,0; 625,0; 626,0; 627,0; 628,0;
          629,0; 630,0; 631,0; 632,0; 633,0; 634,0; 635,0; 636,0; 637,0; 638,0;
          639,0; 640,0; 641,0; 642,0; 643,0; 644,0; 645,0; 646,2; 647,4.5; 648,
          7.8; 649,10.2; 650,12.5; 651,14; 652,15.3; 653,17.5; 654,19.6; 655,21;
          656,22.2; 657,23.3; 658,24.5; 659,25.3; 660,25.6; 661,26; 662,26.1; 663,
          26.2; 664,26.2; 665,26.4; 666,26.5; 667,26.5; 668,26; 669,25.5; 670,
          23.6; 671,21.4; 672,18.5; 673,16.4; 674,14.5; 675,11.6; 676,8.7; 677,
          5.8; 678,3.5; 679,2; 680,0; 681,0; 682,0; 683,0; 684,0; 685,0; 686,0;
          687,0; 688,0; 689,0; 690,0; 691,0; 692,0; 693,0; 694,1.4; 695,3.3; 696,
          4.4; 697,6.5; 698,9.2; 699,11.3; 700,13.5; 701,14.6; 702,16.4; 703,16.7;
          704,16.5; 705,16.5; 706,18.2; 707,19.2; 708,20.1; 709,21.5; 710,22.5;
          711,22.5; 712,22.1; 713,22.7; 714,23.3; 715,23.5; 716,22.5; 717,21.6;
          718,20.5; 719,18; 720,15; 721,12; 722,9; 723,6.2; 724,4.5; 725,3; 726,
          2.1; 727,0.5; 728,0.5; 729,3.2; 730,6.5; 731,9.6; 732,12.5; 733,14; 734,
          16; 735,18; 736,19.6; 737,21.5; 738,23.1; 739,24.5; 740,25.5; 741,26.5;
          742,27.1; 743,27.6; 744,27.9; 745,28.3; 746,28.6; 747,28.6; 748,28.3;
          749,28.2; 750,28; 751,27.5; 752,26.8; 753,25.5; 754,23.5; 755,21.5; 756,
          19; 757,16.5; 758,14.9; 759,12.5; 760,9.4; 761,6.2; 762,3; 763,1.5; 764,
          1.5; 765,0.5; 766,0; 767,3; 768,6.3; 769,9.6; 770,12.9; 771,15.8; 772,
          17.5; 773,18.4; 774,19.5; 775,20.7; 776,22; 777,23.2; 778,25; 779,26.5;
          780,27.5; 781,28; 782,28.3; 783,28.9; 784,28.9; 785,28.9; 786,28.8; 787,
          28.5; 788,28.3; 789,28.3; 790,28.3; 791,28.2; 792,27.6; 793,27.5; 794,
          27.5; 795,27.5; 796,27.5; 797,27.5; 798,27.5; 799,27.6; 800,28; 801,
          28.5; 802,30; 803,31; 804,32; 805,33; 806,33; 807,33.6; 808,34; 809,
          34.3; 810,34.2; 811,34; 812,34; 813,33.9; 814,33.6; 815,33.1; 816,33;
          817,32.5; 818,32; 819,31.9; 820,31.6; 821,31.5; 822,30.6; 823,30; 824,
          29.9; 825,29.9; 826,29.9; 827,29.9; 828,29.6; 829,29.5; 830,29.5; 831,
          29.3; 832,28.9; 833,28.2; 834,27.7; 835,27; 836,25.5; 837,23.7; 838,22;
          839,20.5; 840,19.2; 841,19.2; 842,20.1; 843,20.9; 844,21.4; 845,22; 846,
          22.6; 847,23.2; 848,24; 849,25; 850,26; 851,26.6; 852,26.6; 853,26.8;
          854,27; 855,27.2; 856,27.8; 857,28.1; 858,28.8; 859,28.9; 860,29; 861,
          29.1; 862,29; 863,28.1; 864,27.5; 865,27; 866,25.8; 867,25; 868,24.5;
          869,24.8; 870,25.1; 871,25.5; 872,25.7; 873,26.2; 874,26.9; 875,27.5;
          876,27.8; 877,28.4; 878,29; 879,29.2; 880,29.1; 881,29; 882,28.9; 883,
          28.5; 884,28.1; 885,28; 886,28; 887,27.6; 888,27.2; 889,26.6; 890,27;
          891,27.5; 892,27.8; 893,28; 894,27.8; 895,28; 896,28; 897,28; 898,27.7;
          899,27.4; 900,26.9; 901,26.6; 902,26.5; 903,26.5; 904,26.5; 905,26.3;
          906,26.2; 907,26.2; 908,25.9; 909,25.6; 910,25.6; 911,25.9; 912,25.8;
          913,25.5; 914,24.6; 915,23.5; 916,22.2; 917,21.6; 918,21.6; 919,21.7;
          920,22.6; 921,23.4; 922,24; 923,24.2; 924,24.4; 925,24.9; 926,25.1; 927,
          25.2; 928,25.3; 929,25.5; 930,25.2; 931,25; 932,25; 933,25; 934,24.7;
          935,24.5; 936,24.3; 937,24.3; 938,24.5; 939,25; 940,25; 941,24.6; 942,
          24.6; 943,24.1; 944,24.5; 945,25.1; 946,25.6; 947,25.1; 948,24; 949,22;
          950,20.1; 951,16.9; 952,13.6; 953,10.3; 954,7; 955,3.7; 956,0.4; 957,0;
          958,0; 959,0; 960,2; 961,5.3; 962,8.6; 963,11.9; 964,15.2; 965,17.5;
          966,18.6; 967,20; 968,21.1; 969,22; 970,23; 971,24.5; 972,26.3; 973,
          27.5; 974,28.1; 975,28.4; 976,28.5; 977,28.5; 978,28.5; 979,27.7; 980,
          27.5; 981,27.2; 982,26.8; 983,26.5; 984,26; 985,25.7; 986,25.2; 987,24;
          988,22; 989,21.5; 990,21.5; 991,21.8; 992,22.5; 993,23; 994,22.8; 995,
          22.8; 996,23; 997,22.7; 998,22.7; 999,22.7; 1000,23.5; 1001,24; 1002,
          24.6; 1003,24.8; 1004,25.1; 1005,25.5; 1006,25.6; 1007,25.5; 1008,25;
          1009,24.1; 1010,23.7; 1011,23.2; 1012,22.9; 1013,22.5; 1014,22; 1015,
          21.6; 1016,20.5; 1017,17.5; 1018,14.2; 1019,10.9; 1020,7.6; 1021,4.3;
          1022,1; 1023,0; 1024,0; 1025,0; 1026,0; 1027,0; 1028,0; 1029,0; 1030,0;
          1031,0; 1032,0; 1033,0; 1034,0; 1035,0; 1036,0; 1037,0; 1038,0; 1039,0;
          1040,0; 1041,0; 1042,0; 1043,0; 1044,0; 1045,0; 1046,0; 1047,0; 1048,0;
          1049,0; 1050,0; 1051,0; 1052,0; 1053,1.2; 1054,4; 1055,7.3; 1056,10.6;
          1057,13.9; 1058,17; 1059,18.5; 1060,20; 1061,21.8; 1062,23; 1063,24;
          1064,24.8; 1065,25.6; 1066,26.5; 1067,26.8; 1068,27.4; 1069,27.9; 1070,
          28.3; 1071,28; 1072,27.5; 1073,27; 1074,27; 1075,26.3; 1076,24.5; 1077,
          22.5; 1078,21.5; 1079,20.6; 1080,18; 1081,15; 1082,12.3; 1083,11.1;
          1084,10.6; 1085,10; 1086,9.5; 1087,9.1; 1088,8.7; 1089,8.6; 1090,8.8;
          1091,9; 1092,8.7; 1093,8.6; 1094,8; 1095,7; 1096,5; 1097,4.2; 1098,2.6;
          1099,1; 1100,0; 1101,0.1; 1102,0.6; 1103,1.6; 1104,3.6; 1105,6.9; 1106,
          10; 1107,12.8; 1108,14; 1109,14.5; 1110,16; 1111,18.1; 1112,20; 1113,21;
          1114,21.2; 1115,21.3; 1116,21.4; 1117,21.7; 1118,22.5; 1119,23; 1120,
          23.8; 1121,24.5; 1122,25; 1123,24.9; 1124,24.8; 1125,25; 1126,25.4;
          1127,25.8; 1128,26; 1129,26.4; 1130,26.6; 1131,26.9; 1132,27; 1133,27;
          1134,27; 1135,26.9; 1136,26.8; 1137,26.8; 1138,26.5; 1139,26.4; 1140,26;
          1141,25.5; 1142,24.6; 1143,23.5; 1144,21.5; 1145,20; 1146,17.5; 1147,16;
          1148,14; 1149,10.7; 1150,7.4; 1151,4.1; 1152,0.8; 1153,0; 1154,0; 1155,
          0; 1156,0; 1157,0; 1158,0; 1159,0; 1160,0; 1161,0; 1162,0; 1163,0; 1164,
          0; 1165,0; 1166,0; 1167,0; 1168,0; 1169,2.1; 1170,5.4; 1171,8.7; 1172,
          12; 1173,15.3; 1174,18.6; 1175,21.1; 1176,23; 1177,23.5; 1178,23; 1179,
          22.5; 1180,20; 1181,16.7; 1182,13.4; 1183,10.1; 1184,6.8; 1185,3.5;
          1186,0.2; 1187,0; 1188,0; 1189,0; 1190,0; 1191,0; 1192,0; 1193,0; 1194,
          0; 1195,0; 1196,0; 1197,0.2; 1198,1.5; 1199,3.5; 1200,6.5; 1201,9.8;
          1202,12; 1203,12.9; 1204,13; 1205,12.6; 1206,12.8; 1207,13.1; 1208,13.1;
          1209,14; 1210,15.5; 1211,17; 1212,18.6; 1213,19.7; 1214,21; 1215,21.5;
          1216,21.8; 1217,21.8; 1218,21.5; 1219,21.2; 1220,21.5; 1221,21.8; 1222,
          22; 1223,21.9; 1224,21.7; 1225,21.5; 1226,21.5; 1227,21.4; 1228,20.1;
          1229,19.5; 1230,19.2; 1231,19.6; 1232,19.8; 1233,20; 1234,19.5; 1235,
          17.5; 1236,15.5; 1237,13; 1238,10; 1239,8; 1240,6; 1241,4; 1242,2.5;
          1243,0.7; 1244,0; 1245,0; 1246,0; 1247,0; 1248,0; 1249,0; 1250,0; 1251,
          0; 1252,1; 1253,1; 1254,1; 1255,1; 1256,1; 1257,1.6; 1258,3; 1259,4;
          1260,5; 1261,6.3; 1262,8; 1263,10; 1264,10.5; 1265,9.5; 1266,8.5; 1267,
          7.6; 1268,8.8; 1269,11; 1270,14; 1271,17; 1272,19.5; 1273,21; 1274,21.8;
          1275,22.2; 1276,23; 1277,23.6; 1278,24.1; 1279,24.5; 1280,24.5; 1281,24;
          1282,23.5; 1283,23.5; 1284,23.5; 1285,23.5; 1286,23.5; 1287,23.5; 1288,
          24; 1289,24.1; 1290,24.5; 1291,24.7; 1292,25; 1293,25.4; 1294,25.6;
          1295,25.7; 1296,26; 1297,26.2; 1298,27; 1299,27.8; 1300,28.3; 1301,29;
          1302,29.1; 1303,29; 1304,28; 1305,24.7; 1306,21.4; 1307,18.1; 1308,14.8;
          1309,11.5; 1310,8.2; 1311,4.9; 1312,1.6; 1313,0; 1314,0; 1315,0; 1316,0;
          1317,0; 1318,0; 1319,0; 1320,0; 1321,0; 1322,0; 1323,0; 1324,0; 1325,0;
          1326,0; 1327,0; 1328,0; 1329,0; 1330,0; 1331,0; 1332,0; 1333,0; 1334,0;
          1335,0; 1336,0; 1337,0; 1338,1.5; 1339,4.8; 1340,8.1; 1341,11.4; 1342,
          13.2; 1343,15.1; 1344,16.8; 1345,18.3; 1346,19.5; 1347,20.3; 1348,21.3;
          1349,21.9; 1350,22.1; 1351,22.4; 1352,22; 1353,21.6; 1354,21.1; 1355,
          20.5; 1356,20; 1357,19.6; 1358,18.5; 1359,17.5; 1360,16.5; 1361,15.5;
          1362,14; 1363,11; 1364,8; 1365,5.2; 1366,2.5; 1367,0; 1368,0; 1369,0])
      annotation (Placement(transformation(extent={{-154,70},{-134,90}})));
    Project2_PriusDrivetrain.Ecvt ecvt
      annotation (Placement(transformation(extent={{16,26},{36,46}})));
    Modelica.Mechanics.Rotational.Components.IdealGear FinalDrive(ratio=4.113)
      annotation (Placement(transformation(extent={{48,28},{68,48}})));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-180,12},{-160,32}})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor1
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={86,-4})));
    Modelica.Blocks.Math.Gain mstomph(k=2.237) annotation (Placement(
          transformation(
          extent={{-7,-7},{7,7}},
          rotation=180,
          origin={-11,-5})));
    Modelica.Blocks.Continuous.PID PID(
      k=32,
      Ti=735,
      Td=0.06)
            annotation (Placement(transformation(extent={{-92,20},{-72,40}})));
    Modelica.Mechanics.Translational.Components.Vehicle vehicle(
      m=1700,
      J=2,
      R=0.311,
      A=2.22,
      Cd=0.25,
      CrConstant=0.018)
               annotation (Placement(transformation(extent={{80,28},{100,48}})));
    bat_Tsplit bat_Tsplit1
      annotation (Placement(transformation(extent={{2,60},{22,80}})));
    modeselection_Tsplit modeselection_Tsplit1
      annotation (Placement(transformation(extent={{-54,20},{-34,40}})));
    modeinput_Tsplit modeinput_Tsplit1
      annotation (Placement(transformation(extent={{-22,22},{-2,42}})));
  equation
    connect(timeTable.y, feedback.u1) annotation (Line(points={{-133,80},{-122,
            80},{-122,78},{-120,78},{-120,52},{-178,52},{-178,22}},
                                                      color={0,0,127}));
    connect(feedback.u2, mstomph.y) annotation (Line(points={{-170,14},{-170,-5},
            {-18.7,-5}},                          color={0,0,127}));
    connect(FinalDrive.flange_b, vehicle.flangeR)
      annotation (Line(points={{68,38},{80,38}}, color={0,0,0}));
    connect(vehicle.flangeT, speedSensor1.flange) annotation (Line(points={{100,
            38},{104,38},{104,-4},{96,-4}}, color={0,127,0}));
    connect(ecvt.flange_b, FinalDrive.flange_a)
      annotation (Line(points={{36,37.4},{48,38}}, color={0,0,0}));
    connect(mstomph.u, speedSensor1.v) annotation (Line(points={{-2.6,-5},{36.7,
            -5},{36.7,-4},{75,-4}}, color={0,0,127}));
    connect(feedback.y, PID.u) annotation (Line(points={{-161,22},{-102,22},{
            -102,30},{-94,30}}, color={0,0,127}));
    connect(bat_Tsplit1.torque_mg2, ecvt.MG2_Torque) annotation (Line(points={{
            2,78},{-4,78},{-4,44},{6,44},{6,38},{8,38},{8,20},{30.4,20},{30.4,
            26}}, color={0,0,127}));
    connect(bat_Tsplit1.Torque_mg1, ecvt.MG1_Torque) annotation (Line(points={{
            4.6,64.4},{-2,64.4},{-2,42},{8,42},{8,36},{16,36}}, color={0,0,127}));
    connect(PID.y, modeselection_Tsplit1.torque_req_) annotation (Line(points={
            {-71,30},{-64,30},{-64,34},{-66,34},{-66,46},{-54,46},{-54,36}},
          color={0,0,127}));
    connect(timeTable.y, modeselection_Tsplit1.cyclespeed) annotation (Line(
          points={{-133,80},{-64,80},{-64,36},{-54,36}}, color={0,0,127}));
    connect(modeselection_Tsplit1.y, modeinput_Tsplit1.starting) annotation (
        Line(points={{-34,38},{-34,44},{-24,44},{-24,38},{-27.8,38},{-27.8,38.6}},
          color={255,0,255}));
    connect(modeselection_Tsplit1.y1, modeinput_Tsplit1.depleting) annotation (
        Line(points={{-34,32.6},{-34,46},{-24,46},{-24,34},{-27.6,34}}, color={
            255,0,255}));
    connect(modeselection_Tsplit1.y2, modeinput_Tsplit1.sustain) annotation (
        Line(points={{-34,26.8},{-34,48},{-24,48},{-24,30},{-27.6,30},{-27.6,
            29.2}}, color={255,0,255}));
    connect(modeselection_Tsplit1.y3, modeinput_Tsplit1.regen) annotation (Line(
          points={{-34,23.6},{-34,16},{-27.6,16},{-27.6,23.6}}, color={255,0,
            255}));
    connect(PID.y, modeinput_Tsplit1.torque_req) annotation (Line(points={{-71,
            30},{-64,30},{-64,34},{-66,34},{-66,46},{-36,46},{-36,50},{-18.4,50},
            {-18.4,42.8}}, color={0,0,127}));
    connect(modeinput_Tsplit1.MG2_Torque, ecvt.MG2_Torque) annotation (Line(
          points={{-2,40.6},{0,40.6},{0,38},{2,38},{2,20},{30.4,20},{30.4,26}},
          color={0,0,127}));
    connect(modeinput_Tsplit1.ENG_RPM, ecvt.engine_rpm) annotation (Line(points=
           {{-2,36.6},{10,36.6},{10,45.4},{16,45.4}}, color={0,0,127}));
    connect(modeinput_Tsplit1.ENG_Torque, ecvt.engine_torque) annotation (Line(
          points={{-2,32.4},{6,32.4},{6,50},{16,50},{16,41.8}}, color={0,0,127}));
    annotation (experiment(StopTime=1370, __Dymola_Algorithm="Dassl"));
  end T_split;

  model modeinput_Tsplit
    Modelica.Blocks.Math.BooleanToReal booleanToReal
      annotation (Placement(transformation(extent={{-110,56},{-90,76}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{-108,-40},{-88,-20}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal2
      annotation (Placement(transformation(extent={{-110,12},{-90,32}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal3
      annotation (Placement(transformation(extent={{-106,-92},{-86,-72}})));
    Modelica.Blocks.Interfaces.BooleanInput starting
      annotation (Placement(transformation(extent={{-178,46},{-138,86}})));
    Modelica.Blocks.Interfaces.BooleanInput depleting
      annotation (Placement(transformation(extent={{-176,0},{-136,40}})));
    Modelica.Blocks.Interfaces.BooleanInput sustain
      annotation (Placement(transformation(extent={{-176,-48},{-136,-8}})));
    Modelica.Blocks.Interfaces.BooleanInput regen
      annotation (Placement(transformation(extent={{-176,-104},{-136,-64}})));
    Modelica.Blocks.Interfaces.RealInput torque_req annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={-64,108})));
    Modelica.Blocks.Math.Product product1
      annotation (Placement(transformation(extent={{-56,58},{-36,78}})));
    Modelica.Blocks.Interfaces.RealOutput MG2_Torque
      annotation (Placement(transformation(extent={{90,76},{110,96}})));
    Modelica.Blocks.Interfaces.RealOutput ENG_Torque
      annotation (Placement(transformation(extent={{90,-6},{110,14}})));
    Modelica.Blocks.Math.Product product2
      annotation (Placement(transformation(extent={{-44,-92},{-24,-72}})));
    Modelica.Blocks.Math.Product product3
      annotation (Placement(transformation(extent={{-48,-42},{-28,-22}})));
    Modelica.Blocks.Math.Product product4
      annotation (Placement(transformation(extent={{-52,10},{-32,30}})));
    Project2_PriusDrivetrain.Add4 MG2
      annotation (Placement(transformation(extent={{-12,64},{8,84}})));
    Modelica.Blocks.Math.Gain gain1(k=1)
      annotation (Placement(transformation(extent={{24,-42},{44,-22}})));
    Modelica.Blocks.Math.Gain gain2(k=1)
      annotation (Placement(transformation(extent={{22,10},{42,30}})));
    Modelica.Blocks.Math.Gain gain5(k=0.25)
      annotation (Placement(transformation(extent={{-6,-6},{6,6}},
          rotation=90,
          origin={10,-2})));
    Modelica.Blocks.Math.Gain gain6(k=0.75)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=90,
          origin={-3,35})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{52,-6},{72,14}})));
    Modelica.Blocks.Tables.CombiTable1Ds Torque_RPM(table=[0,0; 7.64,1158.69;
          11.24,1143.58; 17.75,1151.13; 21.57,1158.69; 25.39,1158.69; 28.99,
          1158.69; 32.36,1143.58; 35.51,1151.13; 39.1,1151.13; 42.47,1158.69;
          45.84,1158.69; 49.21,1158.69; 53.26,1166.25; 57.75,1166.25; 61.57,
          1181.36; 64.49,1143.58; 67.19,1173.8; 77.53,1302.27; 78.43,1377.83;
          86.29,2163.73; 86.97,2254.41; 97.75,4370.28])
      annotation (Placement(transformation(extent={{34,42},{54,62}})));
    Modelica.Blocks.Interfaces.RealOutput ENG_RPM
      annotation (Placement(transformation(extent={{90,36},{110,56}})));
    Modelica.Blocks.Math.Gain gain3(k=0.1047)
      annotation (Placement(transformation(extent={{64,48},{76,60}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=97.5, uMin=0)
      annotation (Placement(transformation(extent={{66,-72},{86,-52}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=400, uMin=-400)
      annotation (Placement(transformation(extent={{56,78},{76,98}})));
    Modelica.Blocks.Math.Product engine_power
      annotation (Placement(transformation(extent={{22,-98},{42,-78}})));
    Modelica.Blocks.Tables.CombiTable1Ds Torque_Bsfc(table=[0,0; 7.64,
          0.154483347; 11.24,0.205533434; 17.75,0.267454233; 21.57,0.290792173;
          25.39,0.299500605; 28.99,0.332139154; 32.36,0.355161585; 35.51,
          0.368492521; 39.1,0.392717593; 42.47,0.40074792; 45.84,0.424829143;
          49.21,0.44777446; 53.26,0.478705957; 57.75,0.50930868; 61.57,
          0.539457553; 64.49,0.536255175; 67.19,0.561974906; 77.53,0.704716693;
          78.43,0.738540376; 86.29,1.276103535; 86.97,1.311461323; 97.75,
          2.857657989])
      annotation (Placement(transformation(extent={{106,-42},{126,-22}})));
    Modelica.Blocks.Continuous.Integrator integrator(y_start=0)
      annotation (Placement(transformation(extent={{136,-42},{156,-22}})));
    Modelica.Blocks.Math.Gain gain4(k=0.00026417)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=0,
          origin={175,-33})));
  equation
    connect(booleanToReal3.u,regen)  annotation (Line(points={{-108,-82},{-134,
            -82},{-134,-84},{-156,-84}},
                                    color={255,0,255}));
    connect(sustain,booleanToReal1. u) annotation (Line(points={{-156,-28},{
            -134,-28},{-134,-30},{-110,-30}},color={255,0,255}));
    connect(depleting,booleanToReal2. u) annotation (Line(points={{-156,20},{
            -134,20},{-134,22},{-112,22}},color={255,0,255}));
    connect(starting,booleanToReal. u)
      annotation (Line(points={{-158,66},{-112,66}},color={255,0,255}));
    connect(booleanToReal.y,product1. u2) annotation (Line(points={{-89,66},{
            -66,66},{-66,62},{-58,62}}, color={0,0,127}));
    connect(torque_req,product1. u1) annotation (Line(points={{-64,108},{-62,
            108},{-62,74},{-58,74}}, color={0,0,127}));
    connect(product4.u1,torque_req)  annotation (Line(points={{-54,26},{-54,50},
            {-186,50},{-186,108},{-64,108}}, color={0,0,127}));
    connect(booleanToReal2.y,product4. u2) annotation (Line(points={{-89,22},{
            -62,22},{-62,14},{-54,14}}, color={0,0,127}));
    connect(product3.u1,torque_req)  annotation (Line(points={{-50,-26},{-50,0},
            {-174,0},{-174,108},{-64,108}},  color={0,0,127}));
    connect(booleanToReal1.y,product3. u2) annotation (Line(points={{-87,-30},{
            -58,-30},{-58,-38},{-50,-38}}, color={0,0,127}));
    connect(product2.u1,torque_req)  annotation (Line(points={{-46,-76},{-46,
            -54},{-190,-54},{-190,50},{-186,50},{-186,108},{-64,108}},
                                                                  color={0,0,
            127}));
    connect(booleanToReal3.y,product2. u2) annotation (Line(points={{-85,-82},{
            -54,-82},{-54,-88},{-46,-88}}, color={0,0,127}));
    connect(MG2_Torque, MG2_Torque)
      annotation (Line(points={{100,86},{100,86}}, color={0,0,127}));
    connect(gain5.u,gain1. u) annotation (Line(points={{10,-9.2},{10,-32},{22,
            -32}},
          color={0,0,127}));
    connect(gain5.y,MG2. u3) annotation (Line(points={{10,4.6},{10,58},{-20,58},
            {-20,69},{-14,69}},           color={0,0,127}));
    connect(gain6.y,MG2. u2) annotation (Line(points={{-3,42.7},{-3,48},{-24,48},
            {-24,75.5},{-14,75.5}},
                              color={0,0,127}));
    connect(gain2.u,gain6. u) annotation (Line(points={{20,20},{-3,20},{-3,26.6}},
                                               color={0,0,127}));
    connect(product1.y,MG2. u1) annotation (Line(points={{-35,68},{-26,68},{-26,
            82},{-14,82}},                color={0,0,127}));
    connect(product4.y,gain6. u) annotation (Line(points={{-31,20},{-3,20},{-3,
            26.6}},           color={0,0,127}));
    connect(product3.y,gain1. u) annotation (Line(points={{-27,-32},{22,-32}},
                                                                 color={0,0,127}));
    connect(product2.y,MG2. u4) annotation (Line(points={{-23,-82},{-14,-82},{
            -14,65}},               color={0,0,127}));
    connect(gain2.y, add.u1) annotation (Line(points={{43,20},{48,20},{48,10},{
            50,10}}, color={0,0,127}));
    connect(gain1.y, add.u2) annotation (Line(points={{45,-32},{50,-32},{50,-18},
            {38,-18},{38,-2},{50,-2}}, color={0,0,127}));
    connect(ENG_RPM, gain3.y) annotation (Line(points={{100,46},{78,46},{78,54},
            {76.6,54}}, color={0,0,127}));
    connect(Torque_RPM.y[1], gain3.u) annotation (Line(points={{55,52},{60,52},
            {60,54},{62.8,54}}, color={0,0,127}));
    connect(add.y, limiter.u) annotation (Line(points={{73,4},{78,4},{78,-46},{
            56,-46},{56,-62},{64,-62}}, color={0,0,127}));
    connect(limiter.y, ENG_Torque) annotation (Line(points={{87,-62},{90,-62},{
            90,-10},{86,-10},{86,4},{100,4}}, color={0,0,127}));
    connect(Torque_RPM.u, ENG_Torque) annotation (Line(points={{32,52},{26,52},
            {26,34},{86,34},{86,4},{100,4}}, color={0,0,127}));
    connect(MG2.y, limiter1.u) annotation (Line(points={{9,74},{46,74},{46,88},
            {54,88}}, color={0,0,127}));
    connect(limiter1.y, MG2_Torque) annotation (Line(points={{77,88},{86,88},{
            86,86},{100,86}}, color={0,0,127}));
    connect(ENG_Torque, engine_power.u2) annotation (Line(points={{100,4},{86,4},
            {86,-10},{90,-10},{90,-62},{92,-62},{92,-102},{14,-102},{14,-94},{
            20,-94}}, color={0,0,127}));
    connect(ENG_RPM, engine_power.u1) annotation (Line(points={{100,46},{78,46},
            {78,4},{76,4},{76,-44},{48,-44},{48,-74},{14,-74},{14,-82},{20,-82}},
          color={0,0,127}));
    connect(ENG_Torque, Torque_Bsfc.u) annotation (Line(points={{100,4},{86,4},
            {86,-10},{90,-10},{90,-32},{104,-32}}, color={0,0,127}));
    connect(Torque_Bsfc.y[1], integrator.u)
      annotation (Line(points={{127,-32},{134,-32}}, color={0,0,127}));
    connect(integrator.y, gain4.u)
      annotation (Line(points={{157,-32},{166.6,-33}}, color={0,0,127}));
  end modeinput_Tsplit;

  model bat_PTsplit
    Modelica.Electrical.Analog.Basic.RotationalEMF MG2_EMF(k=1.5)
      annotation (Placement(transformation(extent={{12,8},{-8,28}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{40,-28},{60,-8}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
      annotation (Placement(transformation(extent={{-44,44},{-24,64}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque
      annotation (Placement(transformation(extent={{-68,70},{-48,90}})));
    Modelica.Blocks.Interfaces.RealInput Torque_mg1
      annotation (Placement(transformation(extent={{-120,-64},{-80,-24}})));
    Modelica.Blocks.Interfaces.RealInput torque_mg2 annotation (Placement(
          transformation(extent={{-118,62},{-82,98}}),  iconTransformation(
            extent={{-118,62},{-82,98}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=28,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        Ri=0.15),
      SOC(start=0.7, fixed=true))
      annotation (Placement(transformation(extent={{22,36},{42,56}})));
    Modelica.Electrical.Analog.Basic.RotationalEMF MG1_EMF1(k=-0.46)
      annotation (Placement(transformation(extent={{4,-24},{-16,-4}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque1
      annotation (Placement(transformation(extent={{-56,-14},{-36,6}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1000000, uMin=0)
      annotation (Placement(transformation(extent={{-44,-74},{-24,-54}})));
    Modelica.Blocks.Continuous.Filter filter(f_cut=0.1)
      annotation (Placement(transformation(extent={{-82,-72},{-62,-52}})));
  equation
    connect(torqueSensor.flange_b,MG2_EMF. flange) annotation (Line(points={{-24,54},
            {-18,54},{-18,18},{-8,18}},                       color={0,0,0}));
    connect(torque.flange,MG2_EMF. flange) annotation (Line(points={{-48,80},{
            -16,80},{-16,48},{-18,48},{-18,18},{-8,18}},
                                      color={0,0,0}));
    connect(ground.p, cellStack.n) annotation (Line(points={{50,-8},{50,46},{42,
            46}},                            color={0,0,255}));
    connect(torque_mg2, torque.tau)
      annotation (Line(points={{-100,80},{-70,80}}, color={0,0,127}));
    connect(MG2_EMF.n, cellStack.n) annotation (Line(points={{2,8},{2,2},{34,2},
            {34,0},{50,0},{50,46},{42,46}},    color={0,0,255}));
    connect(MG1_EMF1.p, cellStack.p) annotation (Line(points={{-6,-4},{-24,-4},
            {-24,28},{-16,28},{-16,46},{22,46}}, color={0,0,255}));
    connect(torque1.flange, MG1_EMF1.flange) annotation (Line(points={{-36,-4},
            {-26,-4},{-26,-14},{-16,-14}}, color={0,0,0}));
    connect(MG1_EMF1.n, cellStack.n) annotation (Line(points={{-6,-24},{-6,-28},
            {30,-28},{30,2},{34,2},{34,0},{50,0},{50,46},{42,46}}, color={0,0,
            255}));
    connect(MG2_EMF.p, cellStack.p)
      annotation (Line(points={{2,28},{2,46},{22,46}}, color={0,0,255}));
    connect(limiter.y, torque1.tau) annotation (Line(points={{-23,-64},{-18,-64},
            {-18,-28},{-64,-28},{-64,-4},{-58,-4}}, color={0,0,127}));
    connect(limiter.u, filter.y) annotation (Line(points={{-46,-64},{-46,-54},{
            -61,-54},{-61,-62}}, color={0,0,127}));
    connect(Torque_mg1, filter.u) annotation (Line(points={{-100,-44},{-94,-44},
            {-94,-62},{-84,-62}}, color={0,0,127}));
  end bat_PTsplit;

  model modeinput_PTsplit
    Modelica.Blocks.Math.BooleanToReal booleanToReal
      annotation (Placement(transformation(extent={{-82,80},{-62,100}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal1
      annotation (Placement(transformation(extent={{-80,-16},{-60,4}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal2
      annotation (Placement(transformation(extent={{-82,36},{-62,56}})));
    Modelica.Blocks.Math.BooleanToReal booleanToReal3
      annotation (Placement(transformation(extent={{-78,-68},{-58,-48}})));
    Modelica.Blocks.Math.Product product1
      annotation (Placement(transformation(extent={{-28,82},{-8,102}})));
    Modelica.Blocks.Math.Product product2
      annotation (Placement(transformation(extent={{-16,-68},{4,-48}})));
    Modelica.Blocks.Math.Product product3
      annotation (Placement(transformation(extent={{-20,-18},{0,2}})));
    Modelica.Blocks.Math.Product product4
      annotation (Placement(transformation(extent={{-24,34},{-4,54}})));
    Project2_PriusDrivetrain.Add4 MG2
      annotation (Placement(transformation(extent={{16,88},{36,108}})));
    Modelica.Blocks.Math.Gain gain2(k=0.6)
      annotation (Placement(transformation(extent={{30,-126},{50,-106}})));
    Modelica.Blocks.Math.Gain gain5(k=0.2)
      annotation (Placement(transformation(extent={{-9,-9},{9,9}},
          rotation=90,
          origin={59,17})));
    Modelica.Blocks.Math.Gain gain6(k=0.3)
      annotation (Placement(transformation(extent={{-10,-10},{10,10}},
          rotation=90,
          origin={38,56})));
    Modelica.Blocks.Tables.CombiTable1Ds Torque_RPM(table=[0,0; 7.64,1158.69;
          11.24,1143.58; 17.75,1151.13; 21.57,1158.69; 25.39,1158.69; 28.99,
          1158.69; 32.36,1143.58; 35.51,1151.13; 39.1,1151.13; 42.47,1158.69;
          45.84,1158.69; 49.21,1158.69; 53.26,1166.25; 57.75,1166.25; 61.57,
          1181.36; 64.49,1143.58; 67.19,1173.8; 77.53,1302.27; 78.43,1377.83;
          86.29,2163.73; 86.97,2254.41; 97.75,4370.28])
      annotation (Placement(transformation(extent={{64,-78},{84,-58}})));
    Modelica.Blocks.Math.Gain gain3(k=0.1047)
      annotation (Placement(transformation(extent={{92,72},{104,84}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=44730, uMin=0)
      annotation (Placement(transformation(extent={{104,-138},{124,-118}})));
    Modelica.Blocks.Nonlinear.Limiter limiter1(uMax=400, uMin=-400)
      annotation (Placement(transformation(extent={{84,102},{104,122}})));
    Modelica.Blocks.Tables.CombiTable1Ds Torque_Bsfc(table=[0,0; 7.64,
          0.154483347; 11.24,0.205533434; 17.75,0.267454233; 21.57,0.290792173;
          25.39,0.299500605; 28.99,0.332139154; 32.36,0.355161585; 35.51,
          0.368492521; 39.1,0.392717593; 42.47,0.40074792; 45.84,0.424829143;
          49.21,0.44777446; 53.26,0.478705957; 57.75,0.50930868; 61.57,
          0.539457553; 64.49,0.536255175; 67.19,0.561974906; 77.53,0.704716693;
          78.43,0.738540376; 86.29,1.276103535; 86.97,1.311461323; 97.75,
          2.857657989])
      annotation (Placement(transformation(extent={{134,-18},{154,2}})));
    Modelica.Blocks.Continuous.Integrator integrator(y_start=0)
      annotation (Placement(transformation(extent={{164,-18},{184,2}})));
    Modelica.Blocks.Math.Gain gain4(k=0.00026417)
      annotation (Placement(transformation(extent={{-7,-7},{7,7}},
          rotation=0,
          origin={203,-9})));
    Modelica.Blocks.Interfaces.BooleanInput starting
      annotation (Placement(transformation(extent={{-150,70},{-110,110}})));
    Modelica.Blocks.Interfaces.BooleanInput depleting
      annotation (Placement(transformation(extent={{-148,24},{-108,64}})));
    Modelica.Blocks.Interfaces.BooleanInput sustain
      annotation (Placement(transformation(extent={{-148,-24},{-108,16}})));
    Modelica.Blocks.Interfaces.BooleanInput regen
      annotation (Placement(transformation(extent={{-148,-80},{-108,-40}})));
    Modelica.Blocks.Interfaces.RealInput torque_req annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=-90,
          origin={-36,132})));
    Modelica.Blocks.Interfaces.RealOutput MG2_Torque
      annotation (Placement(transformation(extent={{118,100},{138,120}})));
    Modelica.Blocks.Interfaces.RealOutput ENG_Torque
      annotation (Placement(transformation(extent={{118,18},{138,38}})));
    Modelica.Blocks.Interfaces.RealOutput ENG_RPM
      annotation (Placement(transformation(extent={{118,60},{138,80}})));
    Modelica.Blocks.Interfaces.RealInput power_req annotation (Placement(
          transformation(
          extent={{-20,-20},{20,20}},
          rotation=90,
          origin={-34,-210})));
    Modelica.Blocks.Math.Product product5
      annotation (Placement(transformation(extent={{-16,-126},{4,-106}})));
    Modelica.Blocks.Math.Product product6
      annotation (Placement(transformation(extent={{-16,-164},{4,-144}})));
    Modelica.Blocks.Math.Add add
      annotation (Placement(transformation(extent={{66,-138},{86,-118}})));
    Modelica.Blocks.Math.Gain gain1(k=0.8)
      annotation (Placement(transformation(extent={{24,-164},{44,-144}})));
    Modelica.Blocks.Tables.CombiTable1Ds Power_Torque(table=[0,0; 930,7.64;
          1350,11.24; 2140,17.75; 2620,21.57; 3080,25.39; 3520,28.99; 3870,
          32.36; 4280,35.51; 4710,39.1; 5150,42.47; 5560,45.84; 5970,49.21;
          6500,53.26; 7050,57.75; 7620,61.57; 7720,64.49; 8260,67.19; 10570,
          77.53; 11310,78.43; 19550,86.29; 20530,86.97; 44730,97.75])
      annotation (Placement(transformation(extent={{144,-150},{164,-130}})));
  equation
    connect(booleanToReal3.u,regen)  annotation (Line(points={{-80,-58},{-106,
            -58},{-106,-60},{-128,-60}},
                                    color={255,0,255}));
    connect(sustain,booleanToReal1. u) annotation (Line(points={{-128,-4},{-106,
            -4},{-106,-6},{-82,-6}},         color={255,0,255}));
    connect(depleting,booleanToReal2. u) annotation (Line(points={{-128,44},{
            -106,44},{-106,46},{-84,46}}, color={255,0,255}));
    connect(starting,booleanToReal. u)
      annotation (Line(points={{-130,90},{-84,90}}, color={255,0,255}));
    connect(booleanToReal.y,product1. u2) annotation (Line(points={{-61,90},{
            -38,90},{-38,86},{-30,86}}, color={0,0,127}));
    connect(torque_req,product1. u1) annotation (Line(points={{-36,132},{-34,
            132},{-34,98},{-30,98}}, color={0,0,127}));
    connect(product4.u1,torque_req)  annotation (Line(points={{-26,50},{-26,74},
            {-158,74},{-158,132},{-36,132}}, color={0,0,127}));
    connect(booleanToReal2.y,product4. u2) annotation (Line(points={{-61,46},{
            -34,46},{-34,38},{-26,38}}, color={0,0,127}));
    connect(product3.u1,torque_req)  annotation (Line(points={{-22,-2},{-22,24},
            {-146,24},{-146,132},{-36,132}}, color={0,0,127}));
    connect(booleanToReal1.y,product3. u2) annotation (Line(points={{-59,-6},{
            -30,-6},{-30,-14},{-22,-14}},  color={0,0,127}));
    connect(product2.u1,torque_req)  annotation (Line(points={{-18,-52},{-18,
            -30},{-162,-30},{-162,74},{-158,74},{-158,132},{-36,132}},
                                                                  color={0,0,
            127}));
    connect(booleanToReal3.y,product2. u2) annotation (Line(points={{-57,-58},{
            -26,-58},{-26,-64},{-18,-64}}, color={0,0,127}));
    connect(MG2_Torque,MG2_Torque)
      annotation (Line(points={{128,110},{128,110}},
                                                   color={0,0,127}));
    connect(gain5.y,MG2. u3) annotation (Line(points={{59,26.9},{58,26.9},{58,
            76},{8,76},{8,93},{14,93}},   color={0,0,127}));
    connect(gain6.y,MG2. u2) annotation (Line(points={{38,67},{38,74},{6,74},{6,
            99.5},{14,99.5}}, color={0,0,127}));
    connect(product1.y,MG2. u1) annotation (Line(points={{-7,92},{2,92},{2,106},
            {14,106}},                    color={0,0,127}));
    connect(product4.y,gain6. u) annotation (Line(points={{-3,44},{24,44},{24,
            38},{38,38},{38,44}},
                              color={0,0,127}));
    connect(product2.y,MG2. u4) annotation (Line(points={{5,-58},{14,-58},{14,
            89}},                   color={0,0,127}));
    connect(ENG_RPM,gain3. y) annotation (Line(points={{128,70},{106,70},{106,
            78},{104.6,78}},
                        color={0,0,127}));
    connect(Torque_RPM.y[1],gain3. u) annotation (Line(points={{85,-68},{90,-68},
            {90,78},{90.8,78}}, color={0,0,127}));
    connect(MG2.y,limiter1. u) annotation (Line(points={{37,98},{74,98},{74,112},
            {82,112}},color={0,0,127}));
    connect(limiter1.y,MG2_Torque)  annotation (Line(points={{105,112},{114,112},
            {114,110},{128,110}},
                              color={0,0,127}));
    connect(ENG_Torque,Torque_Bsfc. u) annotation (Line(points={{128,28},{114,
            28},{114,14},{118,14},{118,-8},{132,-8}},
                                                   color={0,0,127}));
    connect(Torque_Bsfc.y[1],integrator. u)
      annotation (Line(points={{155,-8},{162,-8}},   color={0,0,127}));
    connect(integrator.y,gain4. u)
      annotation (Line(points={{185,-8},{194.6,-9}},   color={0,0,127}));
    connect(product5.u2, power_req) annotation (Line(points={{-18,-122},{-18,
            -120},{-34,-120},{-34,-210}}, color={0,0,127}));
    connect(product6.u2, power_req) annotation (Line(points={{-18,-160},{-28,
            -160},{-28,-178},{-34,-178},{-34,-210}}, color={0,0,127}));
    connect(product6.u1, product3.u2) annotation (Line(points={{-18,-148},{-54,
            -148},{-54,-14},{-22,-14}}, color={0,0,127}));
    connect(product5.u1, booleanToReal2.y) annotation (Line(points={{-18,-110},
            {-38,-110},{-38,46},{-61,46}}, color={0,0,127}));
    connect(add.u2, gain1.y) annotation (Line(points={{64,-134},{45,-134},{45,
            -154}}, color={0,0,127}));
    connect(product6.y, gain1.u)
      annotation (Line(points={{5,-154},{22,-154}}, color={0,0,127}));
    connect(product5.y, gain2.u)
      annotation (Line(points={{5,-116},{28,-116}}, color={0,0,127}));
    connect(gain2.y, add.u1) annotation (Line(points={{51,-116},{51,-118},{64,
            -118},{64,-122}}, color={0,0,127}));
    connect(product3.y, gain5.u) annotation (Line(points={{1,-8},{58,-8},{58,2},
            {59,2},{59,6.2}}, color={0,0,127}));
    connect(add.y, limiter.u) annotation (Line(points={{87,-128},{87,-120},{102,
            -120},{102,-128}}, color={0,0,127}));
    connect(limiter.y, Power_Torque.u) annotation (Line(points={{125,-128},{134,
            -128},{134,-140},{142,-140}}, color={0,0,127}));
    connect(Torque_RPM.u, Power_Torque.y[1]) annotation (Line(points={{62,-68},
            {198,-68},{198,-140},{165,-140}}, color={0,0,127}));
    connect(ENG_Torque, Power_Torque.y[1]) annotation (Line(points={{128,28},{
            176,28},{176,-140},{165,-140}}, color={0,0,127}));
    annotation ();
  end modeinput_PTsplit;

  model modeselection_PTsplit
    Modelica.Blocks.Math.Gain wheelrpm(k=13.726)
      annotation (Placement(transformation(extent={{-80,54},{-60,74}})));
    Modelica.Blocks.Math.Product powerreq
      annotation (Placement(transformation(extent={{-8,-86},{12,-66}})));
    Modelica.Blocks.Math.Gain wheelrads(k=0.1047)
      annotation (Placement(transformation(extent={{-40,-64},{-20,-44}})));
    Modelica.Blocks.Logical.And starting
      annotation (Placement(transformation(extent={{40,76},{54,90}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold(threshold=150)
      annotation (Placement(transformation(extent={{-8,86},{4,98}})));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold(threshold=
         300)
      annotation (Placement(transformation(extent={{2,12},{14,24}})));
    Modelica.Blocks.Logical.And deplete
      annotation (Placement(transformation(extent={{42,22},{56,36}})));
    Modelica.Blocks.Logical.GreaterEqualThreshold greaterEqualThreshold1(threshold=
         150)
      annotation (Placement(transformation(extent={{-2,36},{10,48}})));
    Modelica.Blocks.Logical.And sustain
      annotation (Placement(transformation(extent={{48,-36},{62,-22}})));
    Modelica.Blocks.Logical.LessEqualThreshold lessEqualThreshold2(threshold=
          300)
      annotation (Placement(transformation(extent={{2,-16},{14,-4}})));
    Modelica.Blocks.Logical.LessThreshold lessThreshold
      annotation (Placement(transformation(extent={{62,-66},{76,-52}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold
      annotation (Placement(transformation(extent={{-26,66},{-16,76}})));
    Modelica.Blocks.Logical.And sustain1
      annotation (Placement(transformation(extent={{20,-50},{34,-36}})));
    Modelica.Blocks.Logical.GreaterThreshold greaterThreshold1
      annotation (Placement(transformation(extent={{-2,-56},{8,-46}})));
    Modelica.Blocks.Interfaces.RealInput cyclespeed
      "Connector of Real input signal" annotation (Placement(transformation(
            extent={{-112,52},{-88,76}}), iconTransformation(extent={{-112,48},
              {-88,72}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,74},{110,94}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y1
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,20},{110,40}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y2
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,-38},{110,-18}})));
    Modelica.Blocks.Interfaces.BooleanOutput
                                    y3
                                      "Connector of Boolean output signal"
      annotation (Placement(transformation(extent={{90,-70},{110,-50}})));
    Modelica.Blocks.Interfaces.RealInput torque_req_
      "Connector of Real input signal" annotation (Placement(transformation(
            extent={{-112,-10},{-88,14}}),iconTransformation(extent={{-112,48},
              {-88,72}})));
    Modelica.Blocks.Interfaces.RealOutput Power_Req
      annotation (Placement(transformation(extent={{90,-100},{110,-80}})));
  equation
    connect(cyclespeed,wheelrpm. u)
      annotation (Line(points={{-100,64},{-82,64}}, color={0,0,127}));
    connect(wheelrads.y,powerreq. u1) annotation (Line(points={{-19,-54},{-12,
            -54},{-12,-64},{-16,-64},{-16,-70},{-10,-70}}, color={0,0,127}));
    connect(wheelrpm.y,wheelrads. u) annotation (Line(points={{-59,64},{-50,64},
            {-50,-54},{-42,-54}}, color={0,0,127}));
    connect(starting.u1,lessEqualThreshold. y) annotation (Line(points={{38.6,83},
            {16,83},{16,92},{4.6,92}},     color={255,0,255}));
    connect(lessEqualThreshold.u,wheelrpm. y) annotation (Line(points={{-9.2,92},
            {-52,92},{-52,70},{-54,70},{-54,64},{-59,64}}, color={0,0,127}));
    connect(starting.y,y)
      annotation (Line(points={{54.7,83},{100,84}}, color={255,0,255}));
    connect(deplete.y,y1)
      annotation (Line(points={{56.7,29},{100,30}}, color={255,0,255}));
    connect(greaterEqualThreshold1.y,deplete. u1) annotation (Line(points={{10.6,42},
            {32,42},{32,29},{40.6,29}},          color={255,0,255}));
    connect(wheelrpm.y,greaterEqualThreshold1. u) annotation (Line(points={{-59,64},
            {-50,64},{-50,42},{-3.2,42}},     color={0,0,127}));
    connect(sustain.y,y2)
      annotation (Line(points={{62.7,-29},{100,-28}}, color={255,0,255}));
    connect(sustain.u1,greaterEqualThreshold1. y) annotation (Line(points={{46.6,
            -29},{34,-29},{34,28},{32,28},{32,42},{10.6,42}},      color={255,0,
            255}));
    connect(lessThreshold.y,y3)  annotation (Line(points={{76.7,-59},{86,-59},{
            86,-60},{100,-60}}, color={255,0,255}));
    connect(greaterEqualThreshold.y,deplete. u2) annotation (Line(points={{14.6,18},
            {40.6,18},{40.6,23.4}},     color={255,0,255}));
    connect(sustain1.y,sustain. u2) annotation (Line(points={{34.7,-43},{46.6,
            -43},{46.6,-34.6}}, color={255,0,255}));
    connect(lessEqualThreshold2.y,sustain1. u1) annotation (Line(points={{14.6,
            -10},{22,-10},{22,-32},{12,-32},{12,-43},{18.6,-43}}, color={255,0,
            255}));
    connect(greaterThreshold1.u,lessEqualThreshold2. u) annotation (Line(points={{-3,-51},
            {-8,-51},{-8,-10},{0.8,-10}},          color={0,0,127}));
    connect(sustain1.u2,greaterThreshold1. y) annotation (Line(points={{18.6,
            -48.6},{12,-48.6},{12,-51},{8.5,-51}}, color={255,0,255}));
    connect(greaterThreshold.y,starting. u2) annotation (Line(points={{-15.5,71},
            {38.6,71},{38.6,77.4}}, color={255,0,255}));
    connect(greaterThreshold.u,greaterEqualThreshold. u)
      annotation (Line(points={{-27,71},{-27,18},{0.8,18}}, color={0,0,127}));
    connect(lessThreshold.u,powerreq. u2) annotation (Line(points={{60.6,-59},{
            28,-59},{28,-82},{-10,-82}}, color={0,0,127}));
    connect(torque_req_,greaterEqualThreshold. u) annotation (Line(points={{-100,2},
            {-64,2},{-64,18},{0.8,18}},           color={0,0,127}));
    connect(torque_req_,powerreq. u2) annotation (Line(points={{-100,2},{-64,2},
            {-64,-82},{-10,-82}},     color={0,0,127}));
    connect(torque_req_,lessEqualThreshold2. u) annotation (Line(points={{-100,2},
            {-64,2},{-64,18},{-6,18},{-6,-10},{0.8,-10}},      color={0,0,127}));
    connect(powerreq.y, Power_Req) annotation (Line(points={{13,-76},{84,-76},{
            84,-90},{100,-90}}, color={0,0,127}));
    annotation ();
  end modeselection_PTsplit;

  model PT_split
    Modelica.Blocks.Sources.TimeTable timeTable(table=[0,0; 1,0; 2,0; 3,0; 4,0;
          5,0; 6,0; 7,0; 8,0; 9,0; 10,0; 11,0; 12,0; 13,0; 14,0; 15,0; 16,0; 17,
          0; 18,0; 19,0; 20,0; 21,3; 22,5.9; 23,8.6; 24,11.5; 25,14.3; 26,16.9;
          27,17.3; 28,18.1; 29,20.7; 30,21.7; 31,22.4; 32,22.5; 33,22.1; 34,
          21.5; 35,20.9; 36,20.4; 37,19.8; 38,17; 39,14.9; 40,14.9; 41,15.2; 42,
          15.5; 43,16; 44,17.1; 45,19.1; 46,21.1; 47,22.7; 48,22.9; 49,22.7; 50,
          22.6; 51,21.3; 52,19; 53,17.1; 54,15.8; 55,15.8; 56,17.7; 57,19.8; 58,
          21.6; 59,23.2; 60,24.2; 61,24.6; 62,24.9; 63,25; 64,24.6; 65,24.5; 66,
          24.7; 67,24.8; 68,24.7; 69,24.6; 70,24.6; 71,25.1; 72,25.6; 73,25.7;
          74,25.4; 75,24.9; 76,25; 77,25.4; 78,26; 79,26; 80,25.7; 81,26.1; 82,
          26.7; 83,27.5; 84,28.6; 85,29.3; 86,29.8; 87,30.1; 88,30.4; 89,30.7;
          90,30.7; 91,30.5; 92,30.4; 93,30.3; 94,30.4; 95,30.8; 96,30.4; 97,
          29.9; 98,29.5; 99,29.8; 100,30.3; 101,30.7; 102,30.9; 103,31; 104,
          30.9; 105,30.4; 106,29.8; 107,29.9; 108,30.2; 109,30.7; 110,31.2; 111,
          31.8; 112,32.2; 113,32.4; 114,32.2; 115,31.7; 116,28.6; 117,25.3; 118,
          22; 119,18.7; 120,15.4; 121,12.1; 122,8.8; 123,5.5; 124,2.2; 125,0;
          126,0; 127,0; 128,0; 129,0; 130,0; 131,0; 132,0; 133,0; 134,0; 135,0;
          136,0; 137,0; 138,0; 139,0; 140,0; 141,0; 142,0; 143,0; 144,0; 145,0;
          146,0; 147,0; 148,0; 149,0; 150,0; 151,0; 152,0; 153,0; 154,0; 155,0;
          156,0; 157,0; 158,0; 159,0; 160,0; 161,0; 162,0; 163,0; 164,3.3; 165,
          6.6; 166,9.9; 167,13.2; 168,16.5; 169,19.8; 170,22.2; 171,24.3; 172,
          25.8; 173,26.4; 174,25.7; 175,25.1; 176,24.7; 177,25; 178,25.2; 179,
          25.4; 180,25.8; 181,27.2; 182,26.5; 183,24; 184,22.7; 185,19.4; 186,
          17.7; 187,17.2; 188,18.1; 189,18.6; 190,20; 191,22.2; 192,24.5; 193,
          27.3; 194,30.5; 195,33.5; 196,36.2; 197,37.3; 198,39.3; 199,40.5; 200,
          42.1; 201,43.5; 202,45.1; 203,46; 204,46.8; 205,47.5; 206,47.5; 207,
          47.3; 208,47.2; 209,47; 210,47; 211,47; 212,47; 213,47; 214,47.2; 215,
          47.4; 216,47.9; 217,48.5; 218,49.1; 219,49.5; 220,50; 221,50.6; 222,
          51; 223,51.5; 224,52.2; 225,53.2; 226,54.1; 227,54.6; 228,54.9; 229,
          55; 230,54.9; 231,54.6; 232,54.6; 233,54.8; 234,55.1; 235,55.5; 236,
          55.7; 237,56.1; 238,56.3; 239,56.6; 240,56.7; 241,56.7; 242,56.5; 243,
          56.5; 244,56.5; 245,56.5; 246,56.5; 247,56.5; 248,56.4; 249,56.1; 250,
          55.8; 251,55.1; 252,54.6; 253,54.2; 254,54; 255,53.7; 256,53.6; 257,
          53.9; 258,54; 259,54.1; 260,54.1; 261,53.8; 262,53.4; 263,53; 264,
          52.6; 265,52.1; 266,52.4; 267,52; 268,51.9; 269,51.7; 270,51.5; 271,
          51.6; 272,51.8; 273,52.1; 274,52.5; 275,53; 276,53.5; 277,54; 278,
          54.9; 279,55.4; 280,55.6; 281,56; 282,56; 283,55.8; 284,55.2; 285,
          54.5; 286,53.6; 287,52.5; 288,51.5; 289,51.5; 290,51.5; 291,51.1; 292,
          50.1; 293,50; 294,50.1; 295,50; 296,49.6; 297,49.5; 298,49.5; 299,
          49.5; 300,49.1; 301,48.6; 302,48.1; 303,47.2; 304,46.1; 305,45; 306,
          43.8; 307,42.6; 308,41.5; 309,40.3; 310,38.5; 311,37; 312,35.2; 313,
          33.8; 314,32.5; 315,31.5; 316,30.6; 317,30.5; 318,30; 319,29; 320,
          27.5; 321,24.8; 322,21.5; 323,20.1; 324,19.1; 325,18.5; 326,17; 327,
          15.5; 328,12.5; 329,10.8; 330,8; 331,4.7; 332,1.4; 333,0; 334,0; 335,
          0; 336,0; 337,0; 338,0; 339,0; 340,0; 341,0; 342,0; 343,0; 344,0; 345,
          0; 346,0; 347,1; 348,4.3; 349,7.6; 350,10.9; 351,14.2; 352,17.3; 353,
          20; 354,22.5; 355,23.7; 356,25.2; 357,26.6; 358,28.1; 359,30; 360,
          30.8; 361,31.6; 362,32.1; 363,32.8; 364,33.6; 365,34.5; 366,34.6; 367,
          34.9; 368,34.8; 369,34.5; 370,34.7; 371,35.5; 372,36; 373,36; 374,36;
          375,36; 376,36; 377,36; 378,36.1; 379,36.4; 380,36.5; 381,36.4; 382,
          36; 383,35.1; 384,34.1; 385,33.5; 386,31.4; 387,29; 388,25.7; 389,23;
          390,20.3; 391,17.5; 392,14.5; 393,12; 394,8.7; 395,5.4; 396,2.1; 397,
          0; 398,0; 399,0; 400,0; 401,0; 402,0; 403,2.6; 404,5.9; 405,9.2; 406,
          12.5; 407,15.8; 408,19.1; 409,22.4; 410,25; 411,25.6; 412,27.5; 413,
          29; 414,30; 415,30.1; 416,30; 417,29.7; 418,29.3; 419,28.8; 420,28;
          421,25; 422,21.7; 423,18.4; 424,15.1; 425,11.8; 426,8.5; 427,5.2; 428,
          1.9; 429,0; 430,0; 431,0; 432,0; 433,0; 434,0; 435,0; 436,0; 437,0;
          438,0; 439,0; 440,0; 441,0; 442,0; 443,0; 444,0; 445,0; 446,0; 447,0;
          448,3.3; 449,6.6; 450,9.9; 451,13.2; 452,16.5; 453,19.8; 454,23.1;
          455,26.4; 456,27.8; 457,29.1; 458,31.5; 459,33; 460,33.6; 461,34.8;
          462,35.1; 463,35.6; 464,36.1; 465,36; 466,36.1; 467,36.2; 468,36; 469,
          35.7; 470,36; 471,36; 472,35.6; 473,35.5; 474,35.4; 475,35.2; 476,
          35.2; 477,35.2; 478,35.2; 479,35.2; 480,35.2; 481,35; 482,35.1; 483,
          35.2; 484,35.5; 485,35.2; 486,35; 487,35; 488,35; 489,34.8; 490,34.6;
          491,34.5; 492,33.5; 493,32; 494,30.1; 495,28; 496,25.5; 497,22.5; 498,
          19.8; 499,16.5; 500,13.2; 501,10.3; 502,7.2; 503,4; 504,1; 505,0; 506,
          0; 507,0; 508,0; 509,0; 510,0; 511,1.2; 512,3.5; 513,5.5; 514,6.5;
          515,8.5; 516,9.6; 517,10.5; 518,11.9; 519,14; 520,16; 521,17.7; 522,
          19; 523,20.1; 524,21; 525,22; 526,23; 527,23.8; 528,24.5; 529,24.9;
          530,25; 531,25; 532,25; 533,25; 534,25; 535,25; 536,25.6; 537,25.8;
          538,26; 539,25.6; 540,25.2; 541,25; 542,25; 543,25; 544,24.4; 545,
          23.1; 546,19.8; 547,16.5; 548,13.2; 549,9.9; 550,6.6; 551,3.3; 552,0;
          553,0; 554,0; 555,0; 556,0; 557,0; 558,0; 559,0; 560,0; 561,0; 562,0;
          563,0; 564,0; 565,0; 566,0; 567,0; 568,0; 569,3.3; 570,6.6; 571,9.9;
          572,13; 573,14.6; 574,16; 575,17; 576,17; 577,17; 578,17.5; 579,17.7;
          580,17.7; 581,17.5; 582,17; 583,16.9; 584,16.6; 585,17; 586,17.1; 587,
          17; 588,16.6; 589,16.5; 590,16.5; 591,16.6; 592,17; 593,17.6; 594,
          18.5; 595,19.2; 596,20.2; 597,21; 598,21.1; 599,21.2; 600,21.6; 601,
          22; 602,22.4; 603,22.5; 604,22.5; 605,22.5; 606,22.7; 607,23.7; 608,
          25.1; 609,26; 610,26.5; 611,27; 612,26.1; 613,22.8; 614,19.5; 615,
          16.2; 616,12.9; 617,9.6; 618,6.3; 619,3; 620,0; 621,0; 622,0; 623,0;
          624,0; 625,0; 626,0; 627,0; 628,0; 629,0; 630,0; 631,0; 632,0; 633,0;
          634,0; 635,0; 636,0; 637,0; 638,0; 639,0; 640,0; 641,0; 642,0; 643,0;
          644,0; 645,0; 646,2; 647,4.5; 648,7.8; 649,10.2; 650,12.5; 651,14;
          652,15.3; 653,17.5; 654,19.6; 655,21; 656,22.2; 657,23.3; 658,24.5;
          659,25.3; 660,25.6; 661,26; 662,26.1; 663,26.2; 664,26.2; 665,26.4;
          666,26.5; 667,26.5; 668,26; 669,25.5; 670,23.6; 671,21.4; 672,18.5;
          673,16.4; 674,14.5; 675,11.6; 676,8.7; 677,5.8; 678,3.5; 679,2; 680,0;
          681,0; 682,0; 683,0; 684,0; 685,0; 686,0; 687,0; 688,0; 689,0; 690,0;
          691,0; 692,0; 693,0; 694,1.4; 695,3.3; 696,4.4; 697,6.5; 698,9.2; 699,
          11.3; 700,13.5; 701,14.6; 702,16.4; 703,16.7; 704,16.5; 705,16.5; 706,
          18.2; 707,19.2; 708,20.1; 709,21.5; 710,22.5; 711,22.5; 712,22.1; 713,
          22.7; 714,23.3; 715,23.5; 716,22.5; 717,21.6; 718,20.5; 719,18; 720,
          15; 721,12; 722,9; 723,6.2; 724,4.5; 725,3; 726,2.1; 727,0.5; 728,0.5;
          729,3.2; 730,6.5; 731,9.6; 732,12.5; 733,14; 734,16; 735,18; 736,19.6;
          737,21.5; 738,23.1; 739,24.5; 740,25.5; 741,26.5; 742,27.1; 743,27.6;
          744,27.9; 745,28.3; 746,28.6; 747,28.6; 748,28.3; 749,28.2; 750,28;
          751,27.5; 752,26.8; 753,25.5; 754,23.5; 755,21.5; 756,19; 757,16.5;
          758,14.9; 759,12.5; 760,9.4; 761,6.2; 762,3; 763,1.5; 764,1.5; 765,
          0.5; 766,0; 767,3; 768,6.3; 769,9.6; 770,12.9; 771,15.8; 772,17.5;
          773,18.4; 774,19.5; 775,20.7; 776,22; 777,23.2; 778,25; 779,26.5; 780,
          27.5; 781,28; 782,28.3; 783,28.9; 784,28.9; 785,28.9; 786,28.8; 787,
          28.5; 788,28.3; 789,28.3; 790,28.3; 791,28.2; 792,27.6; 793,27.5; 794,
          27.5; 795,27.5; 796,27.5; 797,27.5; 798,27.5; 799,27.6; 800,28; 801,
          28.5; 802,30; 803,31; 804,32; 805,33; 806,33; 807,33.6; 808,34; 809,
          34.3; 810,34.2; 811,34; 812,34; 813,33.9; 814,33.6; 815,33.1; 816,33;
          817,32.5; 818,32; 819,31.9; 820,31.6; 821,31.5; 822,30.6; 823,30; 824,
          29.9; 825,29.9; 826,29.9; 827,29.9; 828,29.6; 829,29.5; 830,29.5; 831,
          29.3; 832,28.9; 833,28.2; 834,27.7; 835,27; 836,25.5; 837,23.7; 838,
          22; 839,20.5; 840,19.2; 841,19.2; 842,20.1; 843,20.9; 844,21.4; 845,
          22; 846,22.6; 847,23.2; 848,24; 849,25; 850,26; 851,26.6; 852,26.6;
          853,26.8; 854,27; 855,27.2; 856,27.8; 857,28.1; 858,28.8; 859,28.9;
          860,29; 861,29.1; 862,29; 863,28.1; 864,27.5; 865,27; 866,25.8; 867,
          25; 868,24.5; 869,24.8; 870,25.1; 871,25.5; 872,25.7; 873,26.2; 874,
          26.9; 875,27.5; 876,27.8; 877,28.4; 878,29; 879,29.2; 880,29.1; 881,
          29; 882,28.9; 883,28.5; 884,28.1; 885,28; 886,28; 887,27.6; 888,27.2;
          889,26.6; 890,27; 891,27.5; 892,27.8; 893,28; 894,27.8; 895,28; 896,
          28; 897,28; 898,27.7; 899,27.4; 900,26.9; 901,26.6; 902,26.5; 903,
          26.5; 904,26.5; 905,26.3; 906,26.2; 907,26.2; 908,25.9; 909,25.6; 910,
          25.6; 911,25.9; 912,25.8; 913,25.5; 914,24.6; 915,23.5; 916,22.2; 917,
          21.6; 918,21.6; 919,21.7; 920,22.6; 921,23.4; 922,24; 923,24.2; 924,
          24.4; 925,24.9; 926,25.1; 927,25.2; 928,25.3; 929,25.5; 930,25.2; 931,
          25; 932,25; 933,25; 934,24.7; 935,24.5; 936,24.3; 937,24.3; 938,24.5;
          939,25; 940,25; 941,24.6; 942,24.6; 943,24.1; 944,24.5; 945,25.1; 946,
          25.6; 947,25.1; 948,24; 949,22; 950,20.1; 951,16.9; 952,13.6; 953,
          10.3; 954,7; 955,3.7; 956,0.4; 957,0; 958,0; 959,0; 960,2; 961,5.3;
          962,8.6; 963,11.9; 964,15.2; 965,17.5; 966,18.6; 967,20; 968,21.1;
          969,22; 970,23; 971,24.5; 972,26.3; 973,27.5; 974,28.1; 975,28.4; 976,
          28.5; 977,28.5; 978,28.5; 979,27.7; 980,27.5; 981,27.2; 982,26.8; 983,
          26.5; 984,26; 985,25.7; 986,25.2; 987,24; 988,22; 989,21.5; 990,21.5;
          991,21.8; 992,22.5; 993,23; 994,22.8; 995,22.8; 996,23; 997,22.7; 998,
          22.7; 999,22.7; 1000,23.5; 1001,24; 1002,24.6; 1003,24.8; 1004,25.1;
          1005,25.5; 1006,25.6; 1007,25.5; 1008,25; 1009,24.1; 1010,23.7; 1011,
          23.2; 1012,22.9; 1013,22.5; 1014,22; 1015,21.6; 1016,20.5; 1017,17.5;
          1018,14.2; 1019,10.9; 1020,7.6; 1021,4.3; 1022,1; 1023,0; 1024,0;
          1025,0; 1026,0; 1027,0; 1028,0; 1029,0; 1030,0; 1031,0; 1032,0; 1033,
          0; 1034,0; 1035,0; 1036,0; 1037,0; 1038,0; 1039,0; 1040,0; 1041,0;
          1042,0; 1043,0; 1044,0; 1045,0; 1046,0; 1047,0; 1048,0; 1049,0; 1050,
          0; 1051,0; 1052,0; 1053,1.2; 1054,4; 1055,7.3; 1056,10.6; 1057,13.9;
          1058,17; 1059,18.5; 1060,20; 1061,21.8; 1062,23; 1063,24; 1064,24.8;
          1065,25.6; 1066,26.5; 1067,26.8; 1068,27.4; 1069,27.9; 1070,28.3;
          1071,28; 1072,27.5; 1073,27; 1074,27; 1075,26.3; 1076,24.5; 1077,22.5;
          1078,21.5; 1079,20.6; 1080,18; 1081,15; 1082,12.3; 1083,11.1; 1084,
          10.6; 1085,10; 1086,9.5; 1087,9.1; 1088,8.7; 1089,8.6; 1090,8.8; 1091,
          9; 1092,8.7; 1093,8.6; 1094,8; 1095,7; 1096,5; 1097,4.2; 1098,2.6;
          1099,1; 1100,0; 1101,0.1; 1102,0.6; 1103,1.6; 1104,3.6; 1105,6.9;
          1106,10; 1107,12.8; 1108,14; 1109,14.5; 1110,16; 1111,18.1; 1112,20;
          1113,21; 1114,21.2; 1115,21.3; 1116,21.4; 1117,21.7; 1118,22.5; 1119,
          23; 1120,23.8; 1121,24.5; 1122,25; 1123,24.9; 1124,24.8; 1125,25;
          1126,25.4; 1127,25.8; 1128,26; 1129,26.4; 1130,26.6; 1131,26.9; 1132,
          27; 1133,27; 1134,27; 1135,26.9; 1136,26.8; 1137,26.8; 1138,26.5;
          1139,26.4; 1140,26; 1141,25.5; 1142,24.6; 1143,23.5; 1144,21.5; 1145,
          20; 1146,17.5; 1147,16; 1148,14; 1149,10.7; 1150,7.4; 1151,4.1; 1152,
          0.8; 1153,0; 1154,0; 1155,0; 1156,0; 1157,0; 1158,0; 1159,0; 1160,0;
          1161,0; 1162,0; 1163,0; 1164,0; 1165,0; 1166,0; 1167,0; 1168,0; 1169,
          2.1; 1170,5.4; 1171,8.7; 1172,12; 1173,15.3; 1174,18.6; 1175,21.1;
          1176,23; 1177,23.5; 1178,23; 1179,22.5; 1180,20; 1181,16.7; 1182,13.4;
          1183,10.1; 1184,6.8; 1185,3.5; 1186,0.2; 1187,0; 1188,0; 1189,0; 1190,
          0; 1191,0; 1192,0; 1193,0; 1194,0; 1195,0; 1196,0; 1197,0.2; 1198,1.5;
          1199,3.5; 1200,6.5; 1201,9.8; 1202,12; 1203,12.9; 1204,13; 1205,12.6;
          1206,12.8; 1207,13.1; 1208,13.1; 1209,14; 1210,15.5; 1211,17; 1212,
          18.6; 1213,19.7; 1214,21; 1215,21.5; 1216,21.8; 1217,21.8; 1218,21.5;
          1219,21.2; 1220,21.5; 1221,21.8; 1222,22; 1223,21.9; 1224,21.7; 1225,
          21.5; 1226,21.5; 1227,21.4; 1228,20.1; 1229,19.5; 1230,19.2; 1231,
          19.6; 1232,19.8; 1233,20; 1234,19.5; 1235,17.5; 1236,15.5; 1237,13;
          1238,10; 1239,8; 1240,6; 1241,4; 1242,2.5; 1243,0.7; 1244,0; 1245,0;
          1246,0; 1247,0; 1248,0; 1249,0; 1250,0; 1251,0; 1252,1; 1253,1; 1254,
          1; 1255,1; 1256,1; 1257,1.6; 1258,3; 1259,4; 1260,5; 1261,6.3; 1262,8;
          1263,10; 1264,10.5; 1265,9.5; 1266,8.5; 1267,7.6; 1268,8.8; 1269,11;
          1270,14; 1271,17; 1272,19.5; 1273,21; 1274,21.8; 1275,22.2; 1276,23;
          1277,23.6; 1278,24.1; 1279,24.5; 1280,24.5; 1281,24; 1282,23.5; 1283,
          23.5; 1284,23.5; 1285,23.5; 1286,23.5; 1287,23.5; 1288,24; 1289,24.1;
          1290,24.5; 1291,24.7; 1292,25; 1293,25.4; 1294,25.6; 1295,25.7; 1296,
          26; 1297,26.2; 1298,27; 1299,27.8; 1300,28.3; 1301,29; 1302,29.1;
          1303,29; 1304,28; 1305,24.7; 1306,21.4; 1307,18.1; 1308,14.8; 1309,
          11.5; 1310,8.2; 1311,4.9; 1312,1.6; 1313,0; 1314,0; 1315,0; 1316,0;
          1317,0; 1318,0; 1319,0; 1320,0; 1321,0; 1322,0; 1323,0; 1324,0; 1325,
          0; 1326,0; 1327,0; 1328,0; 1329,0; 1330,0; 1331,0; 1332,0; 1333,0;
          1334,0; 1335,0; 1336,0; 1337,0; 1338,1.5; 1339,4.8; 1340,8.1; 1341,
          11.4; 1342,13.2; 1343,15.1; 1344,16.8; 1345,18.3; 1346,19.5; 1347,
          20.3; 1348,21.3; 1349,21.9; 1350,22.1; 1351,22.4; 1352,22; 1353,21.6;
          1354,21.1; 1355,20.5; 1356,20; 1357,19.6; 1358,18.5; 1359,17.5; 1360,
          16.5; 1361,15.5; 1362,14; 1363,11; 1364,8; 1365,5.2; 1366,2.5; 1367,0;
          1368,0; 1369,0])
      annotation (Placement(transformation(extent={{-150,34},{-130,54}})));
    Project2_PriusDrivetrain.Ecvt ecvt
      annotation (Placement(transformation(extent={{42,4},{62,24}})));
    Modelica.Mechanics.Rotational.Components.IdealGear FinalDrive(ratio=4.113)
      annotation (Placement(transformation(extent={{74,6},{94,26}})));
    Modelica.Blocks.Math.Feedback feedback
      annotation (Placement(transformation(extent={{-118,-4},{-98,16}})));
    Modelica.Mechanics.Translational.Sensors.SpeedSensor speedSensor1
      annotation (Placement(transformation(
          extent={{-10,-10},{10,10}},
          rotation=180,
          origin={112,-26})));
    Modelica.Blocks.Math.Gain mstomph(k=2.237) annotation (Placement(
          transformation(
          extent={{-7,-7},{7,7}},
          rotation=180,
          origin={15,-27})));
    Modelica.Blocks.Continuous.PID PID(
      k=30,
      Ti=5,
      Td=25)
            annotation (Placement(transformation(extent={{-64,-2},{-44,18}})));
    Modelica.Mechanics.Translational.Components.Vehicle vehicle(
      m=1700,
      J=2,
      R=0.311,
      A=2.22,
      Cd=0.25,
      CrConstant=0.018)
               annotation (Placement(transformation(extent={{106,6},{126,26}})));
    Modelica.Blocks.Continuous.Filter filter(f_cut=0.25)
      annotation (Placement(transformation(extent={{-92,-2},{-72,18}})));
    bat_PTsplit bat_PTsplit1
      annotation (Placement(transformation(extent={{36,52},{56,72}})));
    modeselection_PTsplit modeselection_PTsplit1
      annotation (Placement(transformation(extent={{-32,-4},{-12,16}})));
    modeinput_PTsplit modeinput_PTsplit1
      annotation (Placement(transformation(extent={{0,-2},{20,18}})));
  equation
    connect(speedSensor1.v,mstomph. u) annotation (Line(points={{101,-26},{26,
            -26},{26,-27},{23.4,-27}},                   color={0,0,127}));
    connect(feedback.u2,mstomph. y) annotation (Line(points={{-108,-2},{-108,
            -27},{7.3,-27}},                      color={0,0,127}));
    connect(FinalDrive.flange_b,vehicle. flangeR)
      annotation (Line(points={{94,16},{106,16}},color={0,0,0}));
    connect(vehicle.flangeT,speedSensor1. flange) annotation (Line(points={{126,16},
            {130,16},{130,-26},{122,-26}},  color={0,127,0}));
    connect(ecvt.flange_b,FinalDrive. flange_a)
      annotation (Line(points={{62,15.4},{74,16}}, color={0,0,0}));
    connect(feedback.u1, timeTable.y) annotation (Line(points={{-116,6},{-124,6},
            {-124,44},{-129,44}}, color={0,0,127}));
    connect(PID.u, filter.y)
      annotation (Line(points={{-66,8},{-71,8}}, color={0,0,127}));
    connect(feedback.y, filter.u) annotation (Line(points={{-99,6},{-99,-8},{
            -120,-8},{-120,20},{-94,20},{-94,8}}, color={0,0,127}));
    connect(bat_PTsplit1.torque_mg2, ecvt.MG2_Torque) annotation (Line(points={
            {36,70},{30,70},{30,16},{34,16},{34,-2},{56.4,-2},{56.4,4}}, color=
            {0,0,127}));
    connect(bat_PTsplit1.Torque_mg1, ecvt.MG1_Torque) annotation (Line(points={
            {36,57.6},{32,57.6},{32,14},{42,14}}, color={0,0,127}));
    connect(PID.y, modeselection_PTsplit1.torque_req_) annotation (Line(points=
            {{-43,8},{-40,8},{-40,24},{-32,24},{-32,12}}, color={0,0,127}));
    connect(timeTable.y, modeselection_PTsplit1.cyclespeed)
      annotation (Line(points={{-129,44},{-32,44},{-32,12}}, color={0,0,127}));
    connect(PID.y, modeinput_PTsplit1.torque_req) annotation (Line(points={{-43,
            8},{-40,8},{-40,24},{6.4,24},{6.4,21.2}}, color={0,0,127}));
    connect(modeselection_PTsplit1.Power_Req, modeinput_PTsplit1.power_req)
      annotation (Line(points={{-12,-3},{-12,-8},{6.6,-8},{6.6,-13}}, color={0,
            0,127}));
    connect(modeselection_PTsplit1.y, modeinput_PTsplit1.starting) annotation (
        Line(points={{-12,14.4},{-12,22},{-3,22},{-3,17}}, color={255,0,255}));
    connect(modeselection_PTsplit1.y1, modeinput_PTsplit1.depleting)
      annotation (Line(points={{-12,9},{-12,12.4},{-2.8,12.4}}, color={255,0,
            255}));
    connect(modeselection_PTsplit1.y2, modeinput_PTsplit1.sustain)
      annotation (Line(points={{-12,3.2},{-2.8,7.6}}, color={255,0,255}));
    connect(modeselection_PTsplit1.y3, modeinput_PTsplit1.regen) annotation (
        Line(points={{-12,0},{-12,-6},{-2.8,-6},{-2.8,2}}, color={255,0,255}));
    connect(modeinput_PTsplit1.MG2_Torque, ecvt.MG2_Torque) annotation (Line(
          points={{22.8,19},{30,19},{30,16},{34,16},{34,-2},{56.4,-2},{56.4,4}},
          color={0,0,127}));
    connect(modeinput_PTsplit1.ENG_RPM, ecvt.engine_rpm) annotation (Line(
          points={{22.8,15},{36,15},{36,23.4},{42,23.4}}, color={0,0,127}));
    connect(modeinput_PTsplit1.ENG_Torque, ecvt.engine_torque) annotation (Line(
          points={{22.8,10.8},{32,10.8},{32,19.8},{42,19.8}}, color={0,0,127}));
  end PT_split;

  model bat_Tsplit
    Modelica.Electrical.Analog.Basic.RotationalEMF MG2_EMF(k=1.5)
      annotation (Placement(transformation(extent={{38,-4},{18,16}})));
    Modelica.Electrical.Analog.Basic.Ground ground
      annotation (Placement(transformation(extent={{66,-40},{86,-20}})));
    Modelica.Mechanics.Rotational.Sensors.TorqueSensor torqueSensor
      annotation (Placement(transformation(extent={{-18,32},{2,52}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque
      annotation (Placement(transformation(extent={{-42,58},{-22,78}})));
    Modelica.Electrical.Batteries.BatteryStacks.CellStack cellStack(
      Ns=28,
      cellData(
        Qnom(displayUnit="A.h") = 23400,
        OCVmax=7.2,
        Ri=0.15),
      SOC(start=0.7, fixed=true))
      annotation (Placement(transformation(extent={{48,24},{68,44}})));
    Modelica.Electrical.Analog.Basic.RotationalEMF MG1_EMF1(k=-0.46)
      annotation (Placement(transformation(extent={{30,-36},{10,-16}})));
    Modelica.Mechanics.Rotational.Sources.Torque torque1
      annotation (Placement(transformation(extent={{-30,-26},{-10,-6}})));
    Modelica.Blocks.Nonlinear.Limiter limiter(uMax=1000000, uMin=0)
      annotation (Placement(transformation(extent={{-18,-86},{2,-66}})));
    Modelica.Blocks.Interfaces.RealInput Torque_mg1
      annotation (Placement(transformation(extent={{-94,-76},{-54,-36}})));
    Modelica.Blocks.Interfaces.RealInput torque_mg2 annotation (Placement(
          transformation(extent={{-92,50},{-56,86}}),   iconTransformation(
            extent={{-118,62},{-82,98}})));
  equation
    connect(torqueSensor.flange_b,MG2_EMF. flange) annotation (Line(points={{2,42},{
            8,42},{8,6},{18,6}},                              color={0,0,0}));
    connect(torque.flange,MG2_EMF. flange) annotation (Line(points={{-22,68},{
            10,68},{10,36},{8,36},{8,6},{18,6}},
                                      color={0,0,0}));
    connect(ground.p,cellStack. n) annotation (Line(points={{76,-20},{76,34},{
            68,34}},                         color={0,0,255}));
    connect(torque_mg2,torque. tau)
      annotation (Line(points={{-74,68},{-44,68}},  color={0,0,127}));
    connect(MG2_EMF.n,cellStack. n) annotation (Line(points={{28,-4},{28,-10},{
            60,-10},{60,-12},{76,-12},{76,34},{68,34}},
                                               color={0,0,255}));
    connect(MG1_EMF1.p,cellStack. p) annotation (Line(points={{20,-16},{2,-16},
            {2,16},{10,16},{10,34},{48,34}},     color={0,0,255}));
    connect(torque1.flange,MG1_EMF1. flange) annotation (Line(points={{-10,-16},
            {0,-16},{0,-26},{10,-26}},     color={0,0,0}));
    connect(MG1_EMF1.n,cellStack. n) annotation (Line(points={{20,-36},{20,-40},
            {56,-40},{56,-10},{60,-10},{60,-12},{76,-12},{76,34},{68,34}},
                                                                   color={0,0,
            255}));
    connect(MG2_EMF.p,cellStack. p)
      annotation (Line(points={{28,16},{28,34},{48,34}},
                                                       color={0,0,255}));
    connect(limiter.y,torque1. tau) annotation (Line(points={{3,-76},{8,-76},{8,
            -40},{-38,-40},{-38,-16},{-32,-16}},    color={0,0,127}));
    connect(Torque_mg1, limiter.u) annotation (Line(points={{-74,-56},{-26,-56},
            {-26,-76},{-20,-76}}, color={0,0,127}));
    annotation ();
  end bat_Tsplit;
  annotation (uses(Modelica(version="4.0.0")));
end Project2_PriusDrivetrain;
