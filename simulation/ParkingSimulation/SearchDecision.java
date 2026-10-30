/**
 * 文件职责：组装主模型输入，施加桌内方向约束，再采样目标方向
 * 输入：CompactSearch 的特征与访问记忆；输出：经过约束的输入和选中的动作编号。
 * 调用关系：ParkingCore 调用；不加入真实地图坐标。
 */
import java.util.*;
public final class SearchDecision {

 public static float[] observation(CompactSearch m){
  float[] f=m.features();
  for(int a=0;a<32;a++){
   int k=CompactSearch.MAP+a*CompactSearch.FEATURES;
   f[k+25]=(float)m.memory.corridorCost(m,a);
  }
  m.headingGate.mask(m,f);return f;
 }

 public static int choose(CompactSearch m,CompactSearch.Network net,Random rng){
  float[] obs=observation(m);
  // 主模型打分减去 系数×检测带已搜比例；0 表示不加扣分，与原方案逐步一致。
  // 每次决策读取，避免类初始化早于配置；自定义地图未验证，保持不加扣分。
  double weight=Double.parseDouble(System.getProperty("parking.redundancyWeight","0"));
  if(weight<=0||m.localBoundary)return net.choose(obs,rng,false);
  double[] bias=new double[32];
  for(int a=0;a<32;a++)bias[a]=-weight*m.memory.redundancy(m,a);
  return net.choose(obs,bias,rng,false);
 }
}
