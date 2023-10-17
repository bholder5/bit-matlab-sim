extern crate nalgebra as na;

pub type AMat1 = na::SMatrix<f64, 13, 13>;

pub fn init_AMat1_pos() -> AMat1 {
    let a = AMat1::from_row_slice(&[
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,-0.0001000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,
-8.2367787080796617,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0984664756677910,0.0000000000000000,0.0000000000000000,-0.0889371527313366,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,-4.1857390293602226,0.0000000000000000,0.0000000000000000,0.0035998156386987,0.0498588718891957,0.0451957888564567,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,-0.0017453295723558,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,-0.0700946789345333,0.0017453292519943,0.0000000000000000,-0.0531088929854995,0.0074417744800680,0.0007568518454841,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
8.2367787080796617,0.0141098984743545,0.0000000000000000,0.0000000000000000,-0.2653210658771337,-0.0025727138924452,-0.0001523525446187,0.0889371527313366,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,4.1718255602670187,0.0000000000000000,0.0000000000000000,0.0037953418221692,-0.5175741500806124,-0.0450455572708307,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
    ]);
    return a;
}

pub type AMat2 = na::SMatrix<f64, 13, 5>;

pub fn init_AMat2_pos() -> AMat2 {
    let a = AMat2::from_row_slice(&[
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,-0.0015727541646346,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000133840791552,0.0000026941845076,-0.0007965803448869,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000001835535977,0.0000000000000000,
-0.0010249447635342,0.0000412796258665,0.0001156409521258,-1.0000000000000000,0.0000000000000000,
-0.0000412796258665,0.0042278962727886,0.0000014403612810,0.0000000000000000,0.0000000000000000,
-0.0001156409521258,0.0000014403612810,0.0082668321596602,0.0000000000000000,0.0000000000000000,
1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000001000000000,
    ]);
    return a;
}

pub type AMat3 = na::SMatrix<f64, 13, 13>;

pub fn init_AMat3_pos() -> AMat3 {
    let a = AMat3::from_row_slice(&[
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,-0.0001000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,0.0000000000000000,
0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,0.0000000000000000,1.0000000000000000,0.0000000000000000,
-8.2367761117265950,0.0000119874058238,-0.0000000406442956,0.0000000092417176,0.0984655350792244,-0.0000017490428352,0.0002054274365432,-0.0889366194466562,-0.0000042455936650,-0.0000043009352398,0.0003353374306408,-0.0000012881877369,-0.0000000000248491,
-0.7362501129777880,-4.1974130459521337,0.0005647417098167,0.0027050105698902,0.1232146330196493,0.0547382483448982,-0.0821756335230083,-0.0180781147300034,0.0592086408118832,0.0736222720183525,-0.2617068715872248,0.1071295130209920,0.0000003448700162,
-0.0351853127582915,-0.0211503269747728,-0.0000673768976614,0.2129117628814118,-0.0703216540716733,0.0092711676558736,-0.0044461574259963,-0.0200863688176964,-0.0055855603540739,1.1862703345800738,-0.0211756851119445,-0.0055157606771164,0.0000000694078660,
0.1420681298547788,-0.0472499111900144,-0.0000146253404129,-0.2133046569500506,-0.0001517070669867,-0.0025377491407034,0.0236859686953013,0.0227108130802088,-0.0030116672413466,-1.1969610454740343,0.0591570245293834,-0.0100364075501917,-0.0000000168012423,
8.2381030092902456,0.0140987834924500,-0.0000009126513765,-0.0000049257440428,-0.2655348140949291,-0.0025768371609342,-0.0004743689381667,0.0889684088881314,-0.0000957242015047,-0.0001216883592665,-0.0004283349710530,-0.0001902475657277,0.0000000035837227,
7.6407309175755964,4.2929771270333879,-0.0058608325117053,-0.0280723272588008,-1.2375554086098810,-0.5682118031369441,1.2767986994411094,0.1876128788477310,-0.6144613047287685,-0.7640445166021844,2.7159618523421227,-1.1117794828362710,-0.0000035813607171,
-0.0001806107156002,-0.0000244374684723,0.0000183457807217,0.0002363848674964,-0.0002058776848266,0.0000551845771747,0.0022751450696590,-0.0000300747856535,0.0018747198832954,0.0030982360021692,0.0021631090863878,0.0000233467749393,-0.0001003628091449,
    ]);
    return a;
}

pub type AMat4 = na::SMatrix<f64, 13, 5>;

pub fn init_AMat4_pos() -> AMat4 {
    let a = AMat4::from_row_slice(&[
0.0000000000010206,0.0000000001424204,-0.0000165959707459,0.0000000004788549,0.0000000000000000,
-0.0000000000291191,0.0000000008994652,0.0001196852888971,-0.0000000187684830,0.0000000000000000,
-0.0000000130760068,-0.0000000020389070,0.0004816817732024,-0.0000126680671437,0.0000000000000000,
0.0000000127016035,0.0000000005040236,-0.0003486758173647,0.0000122844447852,-0.0000000000000000,
0.0000000000107178,-0.0000000002174714,0.0000328702583227,0.0000000112103183,-0.0000000000000000,
0.0000000008870776,-0.0000000063093536,-0.0023434949044090,0.0000006465585675,0.0000000000000000,
0.0000000000105777,-0.0000000185736305,-0.0000029668705902,0.0000000048230248,0.0000000000000000,
0.0000000001791679,-0.0000000040991530,-0.0007668172281493,0.0000000955812155,0.0000000000000000,
0.0000000184545173,0.0000000014729790,-0.0005622750596831,0.0000178676918511,-0.0000000000000000,
-0.0000000185176908,-0.0000000001725420,0.0005736070779394,-0.0000179152486522,0.0000000000000000,
-0.0000000000390978,0.0000000204930905,0.0000027257555057,-0.0000000098728169,-0.0000000000000000,
-0.0000000025733676,0.0000000262515229,0.0043969634094472,-0.0000021273313742,-0.0000000000000000,
0.0000000000199719,0.0000000000006585,-0.0000004911476740,0.0000000177012456,0.0000000000000000,
    ]);
    return a;
}

pub type AMat5 = na::SMatrix<f64, 5, 13>;

pub fn init_AMat5_pos() -> AMat5 {
    let a = AMat5::from_row_slice(&[
0.0001806107156002,0.0000244374684723,-0.0000183457807217,-0.0002363848674964,0.0002058776848266,-0.0000551845771747,-0.0022751450696590,0.0000300747856535,-0.0018747198832954,-0.0030982360021692,-0.0021631090863878,-0.0000233467749393,0.0001003628091449,
0.0016508321045212,0.0076219196193117,-0.0000258427518453,0.0000058761361558,-0.0005980518683518,-0.0011120891456109,0.1306163678739717,0.0003390769468979,-0.0026994642649878,-0.0027346519478472,0.2132166858503878,-0.0008190649027428,-0.0000000157997244,
-924.2634604766712982,-14.6551390640177512,0.7089572298327117,3.3957747505205984,150.1603941058327791,6.1253993426456637,-159.8973684740890917,-22.6946516192435226,74.3284827797731396,92.4228468053853334,-328.5372627798462304,134.4867610523584176,0.0004346243786381,
0.0351853062999019,0.0211503230925549,-0.0016779523667002,-0.2129117238006989,0.0703216411638831,-0.0092711659541177,0.0044461566098883,0.0200863651307718,0.0055855593288244,-1.1862701168359260,0.0211756812250720,0.0055157596646788,-0.0000000694078532,
-0.0000000000001023,-0.0000000000000050,0.0000000000000008,0.0000000000000020,-0.0000000000001701,0.0000000000000549,-0.0000000000008587,-0.0000000000000052,0.0000000000017081,0.0000000000017192,-0.0000000000009045,0.0000000000000253,0.0000000000158549,
    ]);
    return a;
}

