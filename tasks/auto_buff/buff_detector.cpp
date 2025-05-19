#include "buff_detector.hpp"

#include "tools/logger.hpp"

namespace auto_buff
{
std::vector<std::vector<cv::Point>> get_contours(
  const cv::Mat & binary_img, double area_min, double area_max)
{
  std::vector<std::vector<cv::Point>> contour_list;

  // 查找所有轮廓
  std::vector<std::vector<cv::Point>> contours;
  cv::findContours(binary_img, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

  for (const auto & contour : contours) {
    double area = cv::contourArea(contour);
    // 只考虑面积在范围内的轮廓
    if (area >= area_min && area <= area_max) {
      contour_list.push_back(contour);
      // // 使用多边形逼近来找到近似的四个角点
      // std::vector<cv::Point> approx;
      // cv::approxPolyDP(contour, approx, 0.02 * cv::arcLength(contour, true), true);

      // // 只保留四个顶点的轮廓
      // if (approx.size() == 4) {
      //   contour_list.push_back(approx);
      // }
    }
  }

  return contour_list;
}

Buff_Detector::Buff_Detector(const std::string & config_path) : status_(LOSE), lose_(0)
{
  auto yaml = YAML::LoadFile(config_path);

  enemy_color_ = yaml["enemy_color"].as<std::string>()=="red" ? "blue" : "red";
  auto fsDetect = yaml["detect"];
  contrast_ = fsDetect["contrast"].as<int>();
  brightness_ = fsDetect["brightness"][enemy_color_].as<int>();
  brightness_threshold_ = fsDetect["brightness_threshold"][enemy_color_].as<int>();
  morphology_size_ = fsDetect["morphology_size"][enemy_color_].as<int>();
  dilate_size_ = fsDetect["dilate_size"].as<int>();
  // canny_low_threshold_ = fsDetect["canny_low_threshold"].as<int>();
  // canny_high_threshold_ = fsDetect["canny_high_threshold"].as<int>();

  R_contours_min_area_ = fsDetect["R_contours_min_area"].as<int>();
  R_contours_max_area_ = fsDetect["R_contours_max_area"].as<int>();
  fanblades_head_contours_min_area_ = fsDetect["fanblades_head_contours_min_area"].as<int>();
  fanblades_head_contours_max_area_ = fsDetect["fanblades_head_contours_max_area"].as<int>();
  fanblades_body_contours_min_area_ = fsDetect["fanblades_body_contours_min_area"].as<int>();
  fanblades_body_contours_max_area_ = fsDetect["fanblades_body_contours_max_area"].as<int>();

  auto standard_fanblade_path = fsDetect["standard_fanblade_path"].as<std::string>();
  standard_fanblade = cv::imread(standard_fanblade_path, cv::IMREAD_GRAYSCALE);
  standard_fanblade_size = standard_fanblade.size();
}

std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
{
  /// get filled_image
  cv::Mat handled_img;
  handle_img(bgr_img, handled_img);
  // cv::imshow("handled_img", handled_img);

  cv::cvtColor(handled_img, output, cv::COLOR_GRAY2BGR);

  /// get fanblades
  auto fanblade = detect_fanblades(handled_img);
  if (fanblade.has_value() == false) {
    handle_lose();
    // cv::imshow("Detector", output);
    return std::nullopt;
  }

  /// get r_center
  auto r_center = detect_r_center(fanblade.value(), handled_img);

  /// PowerRune
  PowerRune powerrune(fanblade.value(), r_center);

  /// handle error
  if (powerrune.is_unsolve()) {
    handle_lose();
    cv::imshow("Detector", output);
    return std::nullopt;
  }

  status_ = TRACK;
  lose_ = 0;
  std::optional<PowerRune> P;
  P.emplace(powerrune);
  last_powerrune_ = P;
  // cv::imshow("Detector", output);

  return P;
}

void Buff_Detector::handle_img(const cv::Mat & bgr_img, cv::Mat & handled_img)
{
  // 读取图像并调整亮度和对比度
  bgr_img.convertTo(bgr_img, -1, contrast_, brightness_);

  // 提取颜色通道
  std::vector<cv::Mat> channels;
  cv::split(bgr_img, channels);  // 分离 BGR 通道
  cv::Mat blue = channels[0];    // 蓝色通道
  cv::Mat red = channels[2];     // 红色通道
  // imshow("Blue Channel", blue);
  // imshow("Red Channel", red);

  // 转换为灰度图
  cv::Mat gray_image;
  if (enemy_color_ == "red")
    gray_image = red - blue * 0.7;
  else
    gray_image = blue - red * 0.0;
  // imshow("Gray Image", gray_image);

  // 二值化
  cv::Mat threshold_image;
  cv::threshold(gray_image, threshold_image, brightness_threshold_, 255, cv::THRESH_BINARY);
  cv::imshow("Threshold Image", threshold_image);

  // 闭运算
  cv::Mat element =
    cv::getStructuringElement(cv::MORPH_RECT, cv::Size(morphology_size_, morphology_size_));
  cv::morphologyEx(threshold_image, threshold_image, cv::MORPH_CLOSE, element);

  // 膨胀操作 (让轮廓更完整)
  cv::dilate(threshold_image, threshold_image, element, cv::Point(-1, -1), dilate_size_);

  handled_img = threshold_image;
  cv::imshow("handled img", handled_img);


  // // Step 5: 边缘检测
  // cv::Mat edges;
  // cv::Canny(threshold_image, edges, canny_low_threshold_, canny_high_threshold_);
  // cv::imshow("Edges", edges);
  // // Step 7: 查找轮廓
  // std::vector<std::vector<cv::Point>> contours;
  // cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
  // // Step 8: 轮廓近似
  // for (size_t i = 0; i < contours.size(); i++) {
  //   cv::approxPolyDP(contours[i], contours[i], 1.0, true);
  // }
  // // 创建彩色图像（原图大小，3通道）
  // cv::Mat contour_img = cv::Mat::zeros(edges.size(), CV_8UC3);
  // cv::cvtColor(edges, contour_img, cv::COLOR_GRAY2BGR);  // 转换成 3 通道用于绘制彩色轮廓
  // // 随机颜色绘制轮廓并标注面积
  // for (size_t i = 0; i < contours.size(); i++) {
  //   double area = cv::contourArea(contours[i]);
  //   // 生成随机颜色
  //   cv::Scalar color = cv::Scalar(rand() % 256, rand() % 256, rand() % 256);
  //   // 绘制轮廓
  //   cv::drawContours(contour_img, contours, static_cast<int>(i), color, 2);
  //   // 仅在轮廓至少有1个点时添加文本
  //   if (!contours[i].empty()) {
  //     cv::putText(
  //       contour_img,
  //       std::to_string(static_cast<int>(area)),  // 显示整数面积
  //       contours[i][0],                          // 在轮廓第一个点的位置绘制文字
  //       cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
  //   }
  // }
  // // 显示最终的轮廓图像
  // cv::imshow("Contours", contour_img);
  // // Step 9: 绘制并填充轮廓
  // handled_img = cv::Mat::zeros(edges.size(), CV_8UC1);
  // for (size_t i = 0; i < contours.size(); i++) {
  //   double area = cv::contourArea(contours[i]);
  //   if (
  //     (area > target_contours_min_area_ && area < target_contours_max_area_) ||
  //     (area > R_contours_min_area_ && area < R_contours_max_area_)) {
  //     cv::drawContours(handled_img, contours, static_cast<int>(i), cv::Scalar(255), cv::FILLED);
  //   }
  // }
  // cv::imshow("Filled Contours", handled_img);
}

void Buff_Detector::handle_lose()
{
  lose_++;
  if (lose_ >= LOSE_MAX) {
    status_ = LOSE;
    last_powerrune_ = std::nullopt;
  }
  status_ = TEM_LOSE;
}

std::optional<FanBlade> Buff_Detector::detect_fanblades(const cv::Mat & handled_img)
{
  std::optional<FanBlade> F;

  cv::Rect head_rect;
  std::vector<cv::Rect> head_rects;

  cv::Point2f body_box[4];
  int fanblades_angle = 0;

  /// 扇叶头

  if (!detect_fanblades_head(handled_img, head_rects)) {
    tools::logger()->debug("[Buff_Detector] 未找到扇叶头!");
    return F;
  }

  /// 扇叶杆
  for (auto it = head_rects.begin(); it != head_rects.end();) {
    head_center = (it->tl() + it->br()) / 2;
    head_radius = (it->width + it->height) / 4;
    if (!detect_fanblades_body(handled_img, body_box, fanblades_angle)) {
      it = head_rects.erase(it);
    } else {
      head_rect = *it;
      break;
      // ++it;
    }
  }
  if (head_rects.empty()) {
    tools::logger()->debug("[Buff_Detector] 未找到扇叶杆!");
    return F;
  }
  cv::rectangle(output, head_rect, DETECTOR_COLOR_KEY, 2);
  tools::draw_point(output, head_center, DETECTOR_COLOR_KEY, 2);
  for (int i = 0; i < 4; i++) {
    cv::line(output, body_box[i], body_box[(i + 1) % 4], DETECTOR_COLOR_KEY, 2);
  }

  /// 扇叶

  std::vector<cv::Point2f> kpt;

  if(std::fabs(head_center.y - body_center.y) < 5){
    tools::logger()->debug("[Buff_Detector] 无法确认点位置!");
    return F;
  }


  if (head_center.y < body_center.y) {
    // fanblades_angle -= 180;
    cv::swap(body_box[0], body_box[2]);
    cv::swap(body_box[1], body_box[3]);
  }

  // head_points
  cv::Mat mask = cv::Mat::zeros(handled_img.size(), CV_8U);
  cv::rectangle(mask, head_rect, cv::Scalar(255), cv::FILLED);
  cv::Mat masked_img;
  handled_img.copyTo(masked_img, mask);

  cv::Mat rotation_matrix = cv::getRotationMatrix2D(head_center, fanblades_angle, 1.0);
  cv::Mat rotated_img;
  cv::warpAffine(masked_img, rotated_img, rotation_matrix, handled_img.size());

  // 去除小噪点
  cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
  cv::morphologyEx(rotated_img, rotated_img, cv::MORPH_OPEN, kernel);  // 形态学开运算
  cv::Mat labels, stats, centroids;                                    // 连通域分析
  int num_labels = cv::connectedComponentsWithStats(rotated_img, labels, stats, centroids);

  for (int i = 1; i < num_labels; i++) {
    int area = stats.at<int>(i, cv::CC_STAT_AREA);
    if (area < 50) rotated_img.setTo(0, labels == i);
  }

  // 寻找 `rotated_img` 中扇叶头的四个关键点
  std::vector<cv::Point> non_zero_pixels;
  cv::findNonZero(rotated_img, non_zero_pixels);

  cv::Point2f top = non_zero_pixels[0], bottom = non_zero_pixels[0];
  cv::Point2f left = non_zero_pixels[0], right = non_zero_pixels[0];

  for (const auto & p : non_zero_pixels) {
    if (p.y < top.y) top = p;
    if (p.y > bottom.y) bottom = p;
    if (p.x < left.x) left = p;
    if (p.x > right.x) right = p;
  }

  // 将 `rotated_img` 关键点转换回原图坐标
  std::vector<cv::Point2f> rotated_points = {top, right, bottom, left};
  std::vector<cv::Point2f> head_points;
  cv::Mat inverse_rotation_matrix;
  cv::invertAffineTransform(rotation_matrix, inverse_rotation_matrix);
  cv::transform(rotated_points, head_points, inverse_rotation_matrix);

  for (const auto & pt : head_points) {
    kpt.push_back(pt);
  }
  for (auto & point : body_box) {
    kpt.push_back(point);
  }

  // 可视化关键点
  for (int i = 0; i < kpt.size(); i++) {
    cv::circle(output, kpt[i], 3, DETECTOR_COLOR_KEY, -1);
    cv::putText(
      output, std::to_string(i), kpt[i], cv::FONT_HERSHEY_SIMPLEX, 0.5, DETECTOR_COLOR_KEY, 1);
  }

  FanBlade fanblade(kpt, head_center, _light);
  F.emplace(fanblade);
  return F;
}

bool Buff_Detector::detect_fanblades_head(
  const cv::Mat & handled_img, std::vector<cv::Rect> & head_rects)
{
  auto contours_list =
    get_contours(handled_img, fanblades_head_contours_min_area_, fanblades_head_contours_max_area_);

  /// 扇叶头
  cv::Mat result;
  if (contours_list.empty()) {
    tools::logger()->debug("[Buff_Detector] contours_list is empty!");
  }
  for (const auto & contours : contours_list) {
    cv::Rect bounding_box = cv::boundingRect(contours);
    // 检查矩形是否超出图像范围 跳过不符合宽高比的矩形
    if (
      bounding_box.x < 0 || bounding_box.y < 0 ||
      bounding_box.x + bounding_box.width > handled_img.cols ||
      bounding_box.y + bounding_box.height > handled_img.rows)
      continue;

    double aspect_ratio = (double)bounding_box.width / bounding_box.height;
    if (aspect_ratio < 0.5 || aspect_ratio > 2) continue;

    // 检查矩形的宽高是否在合理范围内，以避免将整个符作为一个闭合轮廓识别
    // todo: 更动态的约束条件
    if (bounding_box.width > 100 || bounding_box.height > 100) continue;

    // 模板匹配
    cv::Mat roi = handled_img(bounding_box);
    cv::resize(roi, roi, standard_fanblade_size, 0, 0, cv::INTER_AREA);
    cv::matchTemplate(roi, standard_fanblade, result, cv::TM_CCOEFF_NORMED);
    double min_val, max_val;
    cv::minMaxLoc(result, &min_val, &max_val);
    cv::cvtColor(roi, roi, cv::COLOR_GRAY2BGR); // 只是为了绘制彩色文字
    tools::draw_text(roi, fmt::format("max_val: {}", max_val), {0, 20}, cv::Scalar(0, 255, 0), 0.7, 1);
    if (max_val > 0.15) {
      head_rects.push_back(bounding_box);
      tools::draw_text(roi, fmt::format("[success match]"), {0, 35}, cv::Scalar(0, 255, 0), 0.7, 1);
    } else {
      tools::draw_text(roi, fmt::format("[failed match]"), {0, 35}, cv::Scalar(0, 0, 255), 0.7, 1);
      cv::imshow("roi_fanblade", roi);
    }

    tools::draw_text(roi, fmt::format("width: {}, height: {}", bounding_box.width, bounding_box.height), {0, 50}, cv::Scalar(255, 0, 0), 0.7, 1);

    // 绘制所有检测的矩形
    cv::rectangle(output, bounding_box, DETECTOR_COLOR_DEBUG, 1);
    cv::putText(
      output, std::to_string(max_val), bounding_box.tl(), cv::FONT_HERSHEY_SIMPLEX, 1.0,
      DETECTOR_COLOR_DEBUG, 1);
  }

  return head_rects.size() > 0;
  // double best_rotation_angle = 0;  // 最佳匹配的旋转角度
  // cv::Mat roi = handled_img(best_match_rect);
  // cv::resize(roi, roi, standard_fanblade_size, 0, 0, cv::INTER_AREA);
  // // cv::normalize(roi, roi, 0, 255, cv::NORM_MINMAX, CV_8U);
  // // cv::imwrite("result.jpg", roi);
  // for (double angle = 0; angle <= 90; angle += 5) {  // 旋转范围：0° ~ 90°，步长1°
  //   cv::Mat rotated_template;
  //   cv::Mat rotation_matrix = cv::getRotationMatrix2D(
  //     cv::Point2f(standard_fanblade.cols / 2, standard_fanblade.rows / 2), angle, 1.0);
  //   cv::warpAffine(standard_fanblade, rotated_template, rotation_matrix, standard_fanblade.size());
  //   cv::matchTemplate(roi, rotated_template, result, cv::TM_CCOEFF_NORMED);
  //   double min_val, max_val;
  //   cv::minMaxLoc(result, &min_val, &max_val);
  //   // 更新最佳匹配
  //   if (max_val > best_match_score) {
  //     best_match_score = max_val;
  //     best_rotation_angle = angle;
  //   }
  // }
}

bool Buff_Detector::detect_fanblades_body(
  const cv::Mat & handled_img, cv::Point2f body_box[4], int & fanblades_angle)
{
  bool found = false;
  auto contours_list =
    get_contours(handled_img, fanblades_body_contours_min_area_, fanblades_body_contours_max_area_);
  for (const auto & contours : contours_list) {
    cv::RotatedRect bounding_box = cv::minAreaRect(contours);
    cv::Rect bbox = cv::boundingRect(contours);

    body_center = bounding_box.center;
    cv::Size2f size = bounding_box.size;

    if (size.width < size.height) {
      bounding_box.angle += 90;  // 调整角度范围，使其始终为 -180°~180°
      std::swap(bounding_box.size.width, bounding_box.size.height);
      size = bounding_box.size;
    }

    bounding_box.points(body_box);
    for (int i = 0; i < 4; i++) {
      cv::line(output, body_box[i], body_box[(i + 1) % 4], DETECTOR_COLOR_DEBUG, 1);
    }

    cv::Mat roi = handled_img(bbox);
    cv::resize(roi, roi, standard_fanblade_size, 0, 0, cv::INTER_AREA);
    tools::draw_text(roi, fmt::format("width: {}, height: {}", bbox.width, bbox.height), {0, 40}, cv::Scalar(255, 0, 0), 0.7, 1);
    cv::imshow("roi_fanblade_body", roi);
    // 检查矩形是否超出图像范围 跳过不符合宽高比的矩形
    float half_width = size.width / 2.0, half_height = size.height / 2.0;
    if (
      body_center.x - half_width < 0 || body_center.y - half_height < 0 ||
      body_center.x + half_width > handled_img.cols ||
      body_center.y + half_height > handled_img.rows) {
      tools::logger()->debug("[Buff_Detector] 超出图像范围!");
      continue;
    }
    if (
      size.width < head_radius * 1.5 || size.width > head_radius * 3.0 ||
      size.height < head_radius * 0.3 || size.height > head_radius * 0.9) {
      tools::logger()->debug(
        "[Buff_Detector] 宽高比不符合要求! width: {}, height: {}, head_radius: {}", size.width,
        size.height, head_radius);  // width: 117.57067, height: 32.267525  46
      continue;
    }

    auto distance = cv::norm(body_center - head_center);
    if (distance < head_radius * 2 || distance > head_radius * 3) {
      tools::logger()->debug("[Buff_Detector] distance:{}head_radius:{}", distance, head_radius);
      continue;
    }

    fanblades_angle =
      std::atan2(head_center.y - body_center.y, head_center.x - body_center.x) * 180 / CV_PI;
    auto angle_gap = int(std::abs(fanblades_angle - bounding_box.angle)) % 180;
    if (angle_gap > 5 && angle_gap < 175) {
      tools::logger()->debug("[Buff_Detector] 宽高比{}", angle_gap);
      continue;
    }
    found = true;
    break;
  }
  return found;
}

cv::Point2f Buff_Detector::detect_r_center(FanBlade & fanblade, const cv::Mat & handled_img)
{
  /// 算出大概位置

  cv::Point2f r_center_t = {0, 0};
  r_center_t += (body_center - head_center) * 2.1 + head_center;

  cv::Mat dilated_img = handled_img.clone();
  cv::Mat mask = cv::Mat::zeros(handled_img.size(), CV_8U);  // mask
  circle(mask, r_center_t, head_radius * 0.8, cv::Scalar(255), -1);
  bitwise_and(dilated_img, mask, dilated_img);  // mask选出大概范围
  tools::draw_point(output, r_center_t, DETECTOR_COLOR_DEBUG, 4);
  cv::circle(output, r_center_t, head_radius * 0.8, DETECTOR_COLOR_DEBUG, 1);

  /// 获取轮廓点,矩阵框筛选  TODO

  auto r_center = r_center_t;
  auto contours_list = get_contours(dilated_img, R_contours_min_area_, R_contours_max_area_);
  double ratio_1 = INF;
  for (auto & it : contours_list) {
    auto rotated_rect = cv::minAreaRect(it);
    double ratio = rotated_rect.size.height > rotated_rect.size.width
                     ? rotated_rect.size.height / rotated_rect.size.width
                     : rotated_rect.size.width / rotated_rect.size.height;
    ratio += cv::norm(rotated_rect.center - r_center_t) / (head_radius * 0.27);
    if (ratio < ratio_1) {
      ratio_1 = ratio;
      r_center = rotated_rect.center;
    }
  }
  tools::draw_point(output, r_center, DETECTOR_COLOR_KEY, 2);
  return r_center;
};

// std::optional<PowerRune> Buff_Detector::detect(cv::Mat & bgr_img)
// {
//   /// onnx 模型检测
//   std::vector<YOLOV8KP::Object> results = MODE_.get_onecandidatebox(bgr_img);
//   /// 处理未获得的情况
//   if (results.empty()) {
//     handle_lose();
//     return std::nullopt;
//   }
//   /// results转扇叶FanBlade
//   std::vector<FanBlade> fanblades;
//   auto result = results[0];
//   fanblades.emplace_back(FanBlade(result.kpt, result.kpt[4], _light));
//   /// 生成PowerRune
//   auto r_center = get_r_center(fanblades, bgr_img);
//   PowerRune powerrune(fanblades, r_center, last_powerrune_);
//   /// handle error
//   if (powerrune.is_unsolve()) {
//     handle_lose();
//     return std::nullopt;
//   }
//   status_ = TRACK;
//   lose_ = 0;
//   std::optional<PowerRune> P;
//   P.emplace(powerrune);
//   last_powerrune_ = P;
//   return P;
// }

}  // namespace auto_buff