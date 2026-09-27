// Обёртка над pluginlib: создание плагина по имени.

#ifndef TRAM_RESERVE_ODOMETRY__CORE__PLUGIN_REGISTRY_HPP_
#define TRAM_RESERVE_ODOMETRY__CORE__PLUGIN_REGISTRY_HPP_

#include <memory>
#include <string>
#include <vector>

#include <pluginlib/class_loader.hpp>

namespace tram
{

/// Создать экземпляр плагина типа интерфейса \p T, зарегистрированного под \p name.
///
/// \param package    категория pluginlib / ROS-пакет, владеющий базовым классом
/// \param base_class полное имя базового класса (напр. "tram::ITractionModel")
/// \param name       имя плагина из конфига (напр. "tram::TableTractionModel")
/// \return shared_ptr на созданный плагин или nullptr, если не найден / ошибка
template<typename T>
std::shared_ptr<T> create_plugin(
  const std::string & package,
  const std::string & base_class,
  const std::string & name)
{
  // ClassLoader — один на тип интерфейса и не уничтожается: плагины живут до выхода.
  try {
    static pluginlib::ClassLoader<T> * loader =
      new pluginlib::ClassLoader<T>(package, base_class);
    if (!loader->isClassAvailable(name)) {
      return nullptr;
    }
    return loader->createSharedInstance(name);
  } catch (const std::exception & e) {
    (void)e;
    return nullptr;
  }
}

/// Список всех имён плагинов, доступных для типа интерфейса \p T.
template<typename T>
std::vector<std::string> list_plugins(const std::string & package, const std::string & base_class)
{
  std::vector<std::string> names;
  try {
    pluginlib::ClassLoader<T> loader(package, base_class);
    const auto all = loader.getDeclaredClasses();
    for (const auto & n : all) {
      names.push_back(n);
    }
  } catch (const std::exception &) {
  }
  return names;
}

}  // namespace tram

#endif  // TRAM_RESERVE_ODOMETRY__CORE__PLUGIN_REGISTRY_HPP_
