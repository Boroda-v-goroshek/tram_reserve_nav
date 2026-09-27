// Модель тягового привода: табличная характеристика f(u, v) — удельная сила на ободе
// (сила / эффективная масса, м/с²) при позиции ручки u (−15…+15) и скорости v
// (maps/drive_map.csv). По скорости — линейная интерполяция, вне диапазона — крайние
// значения. С drv_power_extrap тяга выше последнего узла v_n — по постоянной мощности:
// f(u, v) = f(u, v_n)·v_n/v; торможение за v_n постоянно.
// compute_acceleration() — f(u, v) − w0 без динамики привода; запаздывание, инерция,
// уклон и адаптация массы — в KalmanWheelEstimator.
// Параметры: drv_table_file — путь к таблице, drv_w0 — сопротивление, м/с²,
// drv_power_extrap — закон мощности.

#ifndef TRAM_RESERVE_ODOMETRY__PLUGINS__TABLE_TRACTION_MODEL_HPP_
#define TRAM_RESERVE_ODOMETRY__PLUGINS__TABLE_TRACTION_MODEL_HPP_

#include <string>
#include <vector>

#include "tram_reserve_odometry/core/interfaces.hpp"

namespace tram
{

class TableTractionModel : public ITractionModel
{
public:
  TableTractionModel() = default;

  double compute_acceleration(
    const PreprocessedSample & sample, const ModelParams & params) const override;
  std::string name() const override {return "TableTractionModel";}
  bool has_drive_model(const ModelParams & params) const override;
  double drive_force(double notch, double v, const ModelParams & params) const override;

  /// Загрузить таблицу из CSV (`u,v0,v1,...`; строки с # — комментарии).
  bool load(const std::string & path);
  bool loaded() const {return !notch_.empty();}

private:
  void ensure_loaded(const ModelParams & params) const;

  mutable bool tried_ = false;
  mutable std::vector<double> notch_;            ///< позиции ручки (строки)
  mutable std::vector<double> speed_;            ///< узлы скорости, м/с (столбцы)
  mutable std::vector<std::vector<double>> f_;   ///< f[строка][столбец], м/с²
};

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__PLUGINS__TABLE_TRACTION_MODEL_HPP_
