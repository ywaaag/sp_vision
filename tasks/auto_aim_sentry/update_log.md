# 9月29日
**新写一版针对于前哨站的卡尔曼吧🤯**

# 9月28日
装甲板yaw值相对是准确的，但是distance很不准，或许可以用yaw值去优化distance？
当前版本卡尔曼对前哨站效果很差，或许可以会看下23年大作业

# 9月27日
当装甲板跳变时则不开火，感觉可以提升一点命中率  
感觉手眼标定的重复误差有些大，需要看看原理。统计了下论文实验中使用图片的数量:
>   CALIB_HAND_EYE_TSAI(12组)        
    CALIB_HAND_EYE_PARK(20~30组后收敛,总数据90组)  
    CALIB_HAND_EYE_HORAUD    
    CALIB_HAND_EYE_ANDREFF(33组)    
    CALIB_HAND_EYE_DANIILIDIS(20组)

# 9月26日
添加了该更新日志，不定时更新一些想法和测试记录