#include "path_manager/planning_visualization_data.h"
#include <rclcpp/duration.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace path_manager::visualization {
namespace {
template<class... Args> std::string formatDiagnostic(const char *format, Args... args) {
  const int size = std::snprintf(nullptr, 0, format, args...);
  if (size < 0) return {};
  std::vector<char> data(static_cast<size_t>(size) + 1);
  std::snprintf(data.data(), data.size(), format, args...);
  return std::string(data.data(), static_cast<size_t>(size));
}
void require(bool condition, const char *message) {
  if (!condition) throw std::invalid_argument(message);
}
void validateTrajectory(const poly_traj::Trajectory &trajectory) {
  require(std::isfinite(trajectory.getTotalDuration()), "visualization trajectory duration must be finite");
  for (const auto &piece : trajectory) {
    require(std::isfinite(piece.getDuration()) && piece.getDuration() > 0.0 &&
            piece.getCoeffMat().allFinite(), "visualization trajectory pieces must have positive finite durations and finite coefficients");
  }
}
int checkedInt(double value) {
  require(std::isfinite(value) && value >= std::numeric_limits<int>::min() &&
          value <= std::numeric_limits<int>::max(), "visualization sample count or dimension is not representable");
  return static_cast<int>(value);
}
void validateZones(const RiskView &field) {
  require(field.zones.size() <= static_cast<size_t>(std::numeric_limits<int>::max()), "too many visual risk zones");
  for (const auto &zone : field.zones)
    require(zone.center.allFinite() && std::isfinite(zone.reach) && zone.reach > 0.0 &&
            std::isfinite(zone.peak) && zone.peak > 0.0 && zone.peak <= 1.0,
            "visual risk zones require finite centers, positive reach and peak in (0,1]");
}
void validateRiskGeometry(const RiskSettings &settings) {
  require(std::isfinite(settings.vertical_ratio) && settings.vertical_ratio > 0.0 &&
          std::isfinite(settings.softness) && std::isfinite(settings.threshold),
          "visual risk settings require positive finite vertical ratio and finite softness/threshold");
}
void validateTerrain(const TerrainView &terrain, bool profile = false) {
  if (!terrain.valid) return;
  require(std::isfinite(terrain.resolution) && terrain.resolution > 0.0,
          "visual terrain resolution must be positive and finite");
  require(profile ? static_cast<bool>(terrain.elevation_and_gradient) : static_cast<bool>(terrain.elevation),
          "visual terrain callback is missing");
}
}  // namespace

nav_msgs::msg::Path makeFrontEndPath(const std::vector<Eigen::Vector3d> &route,
    const builtin_interfaces::msg::Time &stamp)
{
    for (const auto &point : route) require(point.allFinite(), "visual route point must be finite");
    nav_msgs::msg::Path path_msg;
    path_msg.header.stamp = stamp;
    path_msg.header.frame_id = "map";
    for (const auto &point : route) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path_msg.header;
        pose.pose.position.x = point.x();
        pose.pose.position.y = point.y();
        pose.pose.position.z = point.z();
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }
    return path_msg;
}

std::optional<visualization_msgs::msg::Marker> makeTube(
    const std::vector<Eigen::Vector3d> &pts, int drone_id, double scale,
    const std::string &ns_prefix, float r, float g, float b, float a,
    const Stamp &stamp)
{
    if (pts.empty()) return std::nullopt;
    require(static_cast<bool>(stamp) && std::isfinite(scale), "tube requires a clock and finite scale");
    for (const auto &point : pts) require(point.allFinite(), "visual tube point must be finite");
    visualization_msgs::msg::Marker m;
    m.header.frame_id = "map";
    m.header.stamp = stamp();
    drone_id = std::max(0, drone_id);
    m.ns = ns_prefix + std::to_string(drone_id);
    m.id = drone_id;
    m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;  // identity quaternion (avoids RViz cull/flicker)
    m.scale.x = m.scale.y = m.scale.z = scale;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.points.reserve(pts.size());
    for (const auto &p : pts) {
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        m.points.push_back(gp);
    }
    return m;
}

std::optional<visualization_msgs::msg::Marker> makeTrajectoryTube(
    const poly_traj::Trajectory &traj, int drone_id, double scale,
    const std::string &ns_prefix, float r, float g, float b, float a,
    const Stamp &stamp)
{
    validateTrajectory(traj);
    const double T = traj.getDurations().sum();
    if (T <= 1e-6) return std::nullopt;
    // 0.1 s sampling plus the exact endpoint — the density the retired
    // bridge node used; dense enough to read as a tube at map scale.
    std::vector<Eigen::Vector3d> pts;
    const double count = T / 0.1;
    require(std::isfinite(count) && count >= 0.0 &&
            static_cast<long double>(count) <=
                static_cast<long double>(pts.max_size() - 2),
            "trajectory tube sample count exceeds vector capacity");
    pts.reserve(static_cast<size_t>(count) + 2);
    for (double t = 0.0; t < T; t += 0.1) pts.push_back(traj.getPos(t));
    pts.push_back(traj.getPos(T - 1e-9));
    return makeTube(pts, drone_id, scale, ns_prefix, r, g, b, a, stamp);
}

visualization_msgs::msg::MarkerArray makeChain(
    const std::vector<poly_traj::Trajectory> &runs,
    const std::vector<Eigen::Vector3d> &junctions,
    const poly_traj::Trajectory *terminal, double scale, const Stamp &stamp)
{
    require(static_cast<bool>(stamp), "chain clock is missing");
    for (const auto &run : runs) validateTrajectory(run);
    if (terminal) validateTrajectory(*terminal);
    if (!runs.empty()) require(std::isfinite(scale), "chain scale must be finite");
    for (const auto &point : junctions) require(point.allFinite(), "visual junction must be finite");
    visualization_msgs::msg::MarkerArray arr;
    {
        visualization_msgs::msg::Marker del;
        del.header.frame_id = "map";
        del.header.stamp = stamp();
        del.action = visualization_msgs::msg::Marker::DELETEALL;
        arr.markers.push_back(del);
    }
    if (runs.empty()) {  // clear-only
        return arr;
    }
    // High-contrast palette, cycled; the terminal phase is always WHITE so
    // the prescribed geometry is unmistakable next to the optimized runs.
    static const float kPal[][3] = {
        {0.12f, 0.65f, 1.00f}, {1.00f, 0.55f, 0.10f}, {0.20f, 0.90f, 0.35f},
        {0.95f, 0.30f, 0.85f}, {1.00f, 0.90f, 0.15f}, {0.35f, 0.35f, 1.00f},
        {0.10f, 0.90f, 0.90f}, {1.00f, 0.35f, 0.30f}, {0.60f, 0.95f, 0.20f},
        {0.80f, 0.55f, 1.00f},
    };
    constexpr size_t kPalN = sizeof(kPal) / sizeof(kPal[0]);
    int id = 0;
    const auto tube = [&](const poly_traj::Trajectory &traj, const float *c,
                          const char *ns) {
        const double T = traj.getTotalDuration();
        if (T <= 1e-6) return;
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = stamp();
        m.ns = ns;
        m.id = id++;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = c[0]; m.color.g = c[1]; m.color.b = c[2];
        m.color.a = 0.9f;
        m.lifetime = rclcpp::Duration(0, 0);
        for (double t = 0.0; t < T; t += 0.1) {
            const Eigen::Vector3d p = traj.getPos(t);
            geometry_msgs::msg::Point q;
            q.x = p.x(); q.y = p.y(); q.z = p.z();
            m.points.push_back(q);
        }
        const Eigen::Vector3d pe = traj.getPos(T - 1e-9);
        geometry_msgs::msg::Point qe;
        qe.x = pe.x(); qe.y = pe.y(); qe.z = pe.z();
        m.points.push_back(qe);
        arr.markers.push_back(m);
    };
    for (size_t i = 0; i < runs.size(); ++i)
        tube(runs[i], kPal[i % kPalN], "chain_seg");
    if (terminal) {
        static const float kWhite[3] = {1.0f, 1.0f, 1.0f};
        tube(*terminal, kWhite, "chain_terminal");
    }
    for (size_t j = 0; j < junctions.size(); ++j) {
        visualization_msgs::msg::Marker s;
        s.header.frame_id = "map";
        s.header.stamp = stamp();
        s.ns = "chain_junction";
        s.id = id++;
        s.type = visualization_msgs::msg::Marker::SPHERE;
        s.action = visualization_msgs::msg::Marker::ADD;
        s.pose.position.x = junctions[j].x();
        s.pose.position.y = junctions[j].y();
        s.pose.position.z = junctions[j].z();
        s.pose.orientation.w = 1.0;
        s.scale.x = s.scale.y = s.scale.z = 3.0 * scale;
        s.color.r = s.color.g = s.color.b = 1.0f;
        s.color.a = 0.95f;
        s.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(s);

        visualization_msgs::msg::Marker t = s;
        t.ns = "chain_junction_label";
        t.id = id++;
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.pose.position.z += 4.0 * scale;
        t.scale.x = t.scale.y = 0.0;
        t.scale.z = 4.0 * scale;
        t.text = "J" + std::to_string(j + 1);
        arr.markers.push_back(t);
    }
    return arr;
}

Prepared<visualization_msgs::msg::MarkerArray> makePipeline(
    const std::vector<Eigen::Vector3d> &shortcut_route,
    const std::vector<Eigen::Vector3d> &inner_points, const Stamp &stamp)
{
    require(static_cast<bool>(stamp), "pipeline clock is missing");
    require(shortcut_route.size() <= static_cast<size_t>(std::numeric_limits<int>::max()), "too many pipeline points");
    for (const auto &point : shortcut_route) require(point.allFinite(), "pipeline point must be finite");
    for (const auto &point : inner_points) require(point.allFinite(), "pipeline point must be finite");

    Prepared<visualization_msgs::msg::MarkerArray> result;
    auto &arr = result.message;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = stamp();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    auto sphere_list = [&](const char *ns, double scale,
                           float r, float g, float b) {
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = stamp();
        m.ns = ns;
        m.id = 0;
        m.type = visualization_msgs::msg::Marker::SPHERE_LIST;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = scale;
        m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 0.95f;
        return m;
    };

    // Shortcut vertices: what the front end committed. Orange, chunky.
    auto sc = sphere_list("shortcut_points", 0.9, 1.0f, 0.55f, 0.10f);
    for (const auto &p : shortcut_route) {
        geometry_msgs::msg::Point q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        sc.points.push_back(q);
    }
    arr.markers.push_back(std::move(sc));

    // Index labels so a bad vertex can be named, not pointed at. Thinned on
    // long routes to keep RViz responsive.
    const size_t n = shortcut_route.size();
    const size_t stride = n > 120 ? (n + 119) / 120 : 1;
    for (size_t i = 0; i < n; i += stride) {
        visualization_msgs::msg::Marker t;
        t.header.frame_id = "map";
        t.header.stamp = stamp();
        t.ns = "shortcut_index";
        t.id = static_cast<int>(i);
        t.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        t.action = visualization_msgs::msg::Marker::ADD;
        t.pose.position.x = shortcut_route[i].x();
        t.pose.position.y = shortcut_route[i].y();
        t.pose.position.z = shortcut_route[i].z() + 1.4;
        t.pose.orientation.w = 1.0;
        t.scale.z = 1.2;
        t.color.r = 1.0f; t.color.g = 0.75f; t.color.b = 0.35f;
        t.color.a = 0.9f;
        t.text = std::to_string(i);
        arr.markers.push_back(std::move(t));
    }

    // MINCO seed: the sparse pieces the optimizer actually starts from. Cyan,
    // small — dense enough that size must stay subtle next to the shortcut.
    auto ip = sphere_list("inner_points", 0.35, 0.15f, 0.80f, 0.95f);
    for (const auto &p : inner_points) {
        geometry_msgs::msg::Point q;
        q.x = p.x(); q.y = p.y(); q.z = p.z();
        ip.points.push_back(q);
    }
    arr.markers.push_back(std::move(ip));
    result.diagnostic = formatDiagnostic("[DEBUG-PIPELINE] published %zu shortcut + %zu inner points",
                        shortcut_route.size(), inner_points.size());
    return result;
}

Prepared<visualization_msgs::msg::Marker> makeTrajectoryRisk(
    const poly_traj::Trajectory &traj, const RiskView &field, const Stamp &stamp)
{
    validateTrajectory(traj);
    require(static_cast<bool>(stamp), "trajectory risk clock is missing");
    Prepared<visualization_msgs::msg::Marker> result;
    auto &m = result.message;
    m.header.frame_id = "map";
    m.header.stamp = stamp();
    m.ns = "traj_risk";
    m.id = 0;
    m.type = visualization_msgs::msg::Marker::LINE_STRIP;
    m.action = visualization_msgs::msg::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = 0.45;  // line width (u); slightly thinner than the main traj
    m.lifetime = rclcpp::Duration(0, 0);

    const double T = traj.getDurations().sum();
    if (T <= 1e-6 || field.zones.empty()) {
        // No zones (or empty traj): publish DELETE so a stale colored line
        // from a previous zoned plan cannot outlive its mission.
        m.action = visualization_msgs::msg::Marker::DELETE;
        return result;
    }
    validateZones(field);
    require(static_cast<bool>(field.value), "risk value callback is missing");
    const int n = std::min(2000, std::max(50, checkedInt(T / 0.5)));
    m.points.reserve(n + 1);
    m.colors.reserve(n + 1);
    for (int i = 0; i <= n; ++i) {
        const double t = T * static_cast<double>(i) / n;
        const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
        // OR-combined effective moat (terrain-masked, same field the
        // optimizer prices): r = 1 - prod(1 - m_i).
        double surv = 1.0;
        for (size_t zi = 0; zi < field.zones.size(); ++zi)
            surv *= (1.0 - field.value(zi, p));
        const double r = 1.0 - surv;
        // Perceptual ramp: sqrt lifts the faint-halo values so "slightly
        // visible" is visibly yellow-ish instead of indistinguishable green.
        const double s = std::sqrt(std::clamp(r, 0.0, 1.0));
        geometry_msgs::msg::Point gp;
        gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
        std_msgs::msg::ColorRGBA c;
        c.a = 0.95f;
        if (s < 0.5) {          // green -> yellow
            c.r = static_cast<float>(2.0 * s);
            c.g = 0.85f;
            c.b = 0.05f;
        } else {                // yellow -> red
            c.r = 0.95f;
            c.g = static_cast<float>(0.85 * 2.0 * (1.0 - s));
            c.b = 0.05f;
        }
        m.points.push_back(gp);
        m.colors.push_back(c);
    }
    // Diagnostic: max experienced moat along the line — if this is 0 while
    // the optimizer's risk_cost was nonzero, the viz field and the priced
    // field have diverged (bug), or the cost was pure barrier/velocity terms.
    {
        double max_r = 0.0; Eigen::Vector3d max_p = Eigen::Vector3d::Zero();
        for (int i = 0; i <= n; ++i) {
            const double t = T * static_cast<double>(i) / n;
            const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
            double surv = 1.0;
            for (size_t zi = 0; zi < field.zones.size(); ++zi)
                surv *= (1.0 - field.value(zi, p));
            if (1.0 - surv > max_r) { max_r = 1.0 - surv; max_p = p; }
        }
        result.diagnostic = formatDiagnostic(
            "[TRAJ-RISK] published %d pts, max experienced moat r=%.5f at "
            "(%.1f, %.1f, %.2f), zones=%zu",
            n + 1, max_r, max_p.x(), max_p.y(), max_p.z(), field.zones.size());
    }
    return result;
}

static bool riskProfileVisibleInterval(
    const RiskView &field, const RiskSettings &settings, size_t zone_index,
    double x, double y, double *lower_z, double *floor_z, double *top_z)
{
    const double nan = std::numeric_limits<double>::quiet_NaN();
    if (lower_z) *lower_z = nan;
    if (floor_z) *floor_z = nan;
    if (top_z) *top_z = nan;
    if (!floor_z || !top_z || zone_index >= field.zones.size())
        return false;

    const auto &zone = field.zones[zone_index];
    if (!(zone.reach > 0.0) || !(zone.peak > 0.0) ||
        !(settings.vertical_ratio > 0.0)) {
        return false;
    }

    const double dx = x - zone.center.x();
    const double dy = y - zone.center.y();
    const double rho2 = dx * dx + dy * dy;
    const double reach2 = zone.reach * zone.reach;
    if (!(rho2 < reach2)) return false;

    const double rv = zone.reach * settings.vertical_ratio;
    const double half_z =
        rv * std::sqrt(std::max(0.0, 1.0 - rho2 / reach2));
    const double ellipsoid_lower = zone.center.z() - half_z;
    const double ellipsoid_upper = zone.center.z() + half_z;

    // Keep the same sea/off-map convention used when risk-zone sources are
    // grounded: finite positive DEM is land; pure water and off-DEM use MSL 0.
    double ground = 0.0;
    float terrain_h = 0.0f, terrain_gx = 0.0f, terrain_gy = 0.0f;
    const bool has_ground_sample =
        field.terrain.valid &&
        field.terrain.elevation_and_gradient(
            x, y, &terrain_h, &terrain_gx, &terrain_gy);
    if (has_ground_sample && std::isfinite(terrain_h) && terrain_h > 0.0f)
        ground = static_cast<double>(terrain_h);

    double visible_floor = std::max(ellipsoid_lower, ground);
    if (!(visible_floor < ellipsoid_upper)) return false;

    // riskVisibilityValue is monotone in raw z. Find its configured contour
    // in raw trajectory-z coordinates, rather than mistaking the effective
    // (grounded) z contour for a trajectory altitude.
    const double threshold = settings.threshold;
    if (threshold >= 1.0) return false;  // no finite visible interval
    if (threshold > 0.0) {
        const double ceiling = field.shadow_ceiling(
            zone_index, Eigen::Vector3d(x, y, zone.center.z()));
        if (std::isfinite(ceiling)) {
            const double w = std::max(1e-6, settings.softness);
            const double z_eff_boundary =
                ceiling + w * std::log(threshold / (1.0 - threshold));
            double raw_z_boundary = z_eff_boundary;

            if (settings.grounded && has_ground_sample) {
                const double grounded_floor =
                    static_cast<double>(terrain_h) +
                    settings.obstacle_clearance;
                const double d =
                    (z_eff_boundary - grounded_floor) / w;
                if (d <= 0.0) {
                    // smoothmax(z, grounded_floor) is strictly above its
                    // asymptote for every finite z: the whole shell is visible.
                    raw_z_boundary =
                        -std::numeric_limits<double>::infinity();
                } else if (d > 50.0) {
                    // log(expm1(d)) = d + log(1-exp(-d)); this form avoids
                    // overflow while retaining the exact inverse.
                    raw_z_boundary =
                        grounded_floor +
                        w * (d + std::log1p(-std::exp(-d)));
                } else {
                    raw_z_boundary =
                        grounded_floor + w * std::log(std::expm1(d));
                }
            }
            visible_floor = std::max(visible_floor, raw_z_boundary);
        }
        // A non-finite horizon is the same full-visibility fallback used by
        // riskVisibilityValue (mask disabled/not ready, source cell, etc.).
    }

    // [SHADOW-FULL] A column whose LOS boundary rises above the ellipsoid cap
    // is FULLY shadowed inside the zone — the safest spot in the footprint,
    // not "no zone". Reporting it as no-interval (old behavior) punched
    // white holes into the panel's zone band that were indistinguishable
    // from uncovered sky. Report a zero-height band AT the cap instead
    // (detection floor == zone top): the red band degenerates to nothing and
    // the shadow-safe fill covers the whole cross-section.
    if (!(visible_floor < ellipsoid_upper)) {
        if (lower_z) *lower_z = ellipsoid_lower;
        *floor_z = ellipsoid_upper;
        *top_z = ellipsoid_upper;
        return true;
    }
    if (lower_z) *lower_z = ellipsoid_lower;
    *floor_z = visible_floor;
    *top_z = ellipsoid_upper;
    return true;
}

Prepared<std_msgs::msg::Float64MultiArray> makeRiskProfile(
    const poly_traj::Trajectory &traj, const RiskView &field,
    const RiskSettings &settings)
{
    validateTrajectory(traj);
    const double T = traj.getTotalDuration();
    Prepared<std_msgs::msg::Float64MultiArray> result;
    auto &msg = result.message;
    if (!(T > 0.0) || !std::isfinite(T) || field.zones.empty()) {  // empty = clear stale profile
        return result;
    }
    validateZones(field);
    validateRiskGeometry(settings);
    validateTerrain(field.terrain, true);
    require(static_cast<bool>(field.value) && static_cast<bool>(field.shadow_ceiling), "risk profile callbacks are missing");
    require(std::isfinite(settings.max_velocity) && std::isfinite(settings.obstacle_clearance), "risk profile settings must be finite");
    const double est_len = std::max(1.0, T * settings.max_velocity);
    const int K = std::clamp(checkedInt(est_len / 1.0), 256, 16384);
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const size_t zone_count = field.zones.size();
    // v4: per-zone TRIPLET (geometric lower, detection floor, geometric top)
    // — the panel needs the cross-section lower bound to paint mid-air
    // shadow-safe pockets between overlapping zones (a gap between zone A's
    // cap and zone B's floor was indistinguishable from uncovered sky).
    const size_t record_width = 5 + 3 * zone_count;

    msg.data.reserve(2 + record_width * static_cast<size_t>(K + 1));
    msg.data.push_back(4.0);
    msg.data.push_back(static_cast<double>(zone_count));
    double s = 0.0;
    Eigen::Vector3d prev = traj.getPos(0.0);
    for (int k = 0; k <= K; ++k) {
        const double t = T * static_cast<double>(k) / K;
        const Eigen::Vector3d p = traj.getPos(std::min(t, T - 1e-9));
        s += (p - prev).head<2>().norm();
        prev = p;

        double survival = 1.0;
        for (size_t zi = 0; zi < zone_count; ++zi)
            survival *= (1.0 - field.value(zi, p));
        const double combined_risk = 1.0 - survival;

        msg.data.push_back(s);
        msg.data.push_back(p.x());
        msg.data.push_back(p.y());
        msg.data.push_back(p.z());
        msg.data.push_back(combined_risk);
        for (size_t zi = 0; zi < zone_count; ++zi) {
            double lower_z = nan;
            double floor_z = nan;
            double top_z = nan;
            (void)riskProfileVisibleInterval(field, settings,
                zi, p.x(), p.y(), &lower_z, &floor_z, &top_z);
            msg.data.push_back(lower_z);
            msg.data.push_back(floor_z);
            msg.data.push_back(top_z);
        }
    }
    result.diagnostic = formatDiagnostic(
        "[RISK-PROFILE] v4: %d samples over %.1f u, zones=%zu, width=%zu",
        K + 1, s, zone_count, record_width);
    return result;
}

Prepared<grid_map_msgs::msg::GridMap> makeRiskHeatmap(
    const RiskView &field, const RiskSettings &settings, const Stamp &stamp)
{
    require(static_cast<bool>(stamp), "heatmap clock is missing");
    validateZones(field);

    // Message-layout contract shared with terrain_publisher.py and
    // TerrainData ([TERRAIN-FRAME]): matrix ROWS span X (mirrored), COLS
    // span Y (mirrored), data[col * rows + row], length_x = rows * res.
    auto makeLayer = [](int rows, int cols) {
        std_msgs::msg::Float32MultiArray arr;
        arr.layout.dim.resize(2);
        arr.layout.dim[0].label = "column_index";
        arr.layout.dim[0].size = static_cast<uint32_t>(cols);
        arr.layout.dim[0].stride = static_cast<uint32_t>(cols) * rows;
        arr.layout.dim[1].label = "row_index";
        arr.layout.dim[1].size = static_cast<uint32_t>(rows);
        arr.layout.dim[1].stride = static_cast<uint32_t>(rows);
        arr.data.assign(static_cast<size_t>(rows) * cols,
                        std::numeric_limits<float>::quiet_NaN());
        return arr;
    };
    Prepared<grid_map_msgs::msg::GridMap> result;
    auto &msg = result.message;
    msg.header.frame_id = "map";
    msg.header.stamp = stamp();
    msg.layers = {"elevation", "color"};

    constexpr double kInf = std::numeric_limits<double>::infinity();
    double x0 = kInf, x1 = -kInf, y0 = kInf, y1 = -kInf;
    for (const auto &zone : field.zones) {
        if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
        x0 = std::min(x0, zone.center.x() - zone.reach);
        x1 = std::max(x1, zone.center.x() + zone.reach);
        y0 = std::min(y0, zone.center.y() - zone.reach);
        y1 = std::max(y1, zone.center.y() + zone.reach);
    }
    const bool hm3d = settings.mode == "heatmap3d";
    if ((settings.mode != "heatmap" && !hm3d) || !(x1 > x0)) {
        // Latched topic: replace any stale heatmap with an all-NaN stub.
        msg.info.resolution = 1.0;
        msg.info.length_x = 1.0;
        msg.info.length_y = 1.0;
        msg.info.pose.position.x = 0.0;
        msg.info.pose.position.y = 0.0;
        msg.info.pose.orientation.w = 1.0;
        msg.data = {makeLayer(1, 1), makeLayer(1, 1)};
        return result;
    }

    validateRiskGeometry(settings);
    validateTerrain(field.terrain);
    require(static_cast<bool>(field.shadow_ceiling), "heatmap shadow callback is missing");
    require(settings.heatmap_max_dim > 0 && std::isfinite(settings.heatmap_agl_max) &&
            std::isfinite(settings.heatmap_offset) && std::isfinite(settings.heatmap_preview_lift), "heatmap settings are invalid");
    const double extent = std::max(x1 - x0, y1 - y0);
    const double res = std::max(
        extent / static_cast<double>(settings.heatmap_max_dim), 0.05);
    const int rows = std::max(1, checkedInt(std::ceil((x1 - x0) / res)));
    const int cols = std::max(1, checkedInt(std::ceil((y1 - y0) / res)));
    const double length_x = rows * res;
    const double length_y = cols * res;
    msg.info.resolution = res;
    msg.info.length_x = length_x;
    msg.info.length_y = length_y;
    msg.info.pose.position.x = x0 + 0.5 * length_x;
    msg.info.pose.position.y = y0 + 0.5 * length_y;
    msg.info.pose.orientation.w = 1.0;
    auto elevation = makeLayer(rows, cols);
    auto color = makeLayer(rows, cols);

    // grid_map_core colorVectorToValue packing (0xRRGGBB reinterpreted as
    // float32). Self-contained — terrain_publisher.py no longer has a color
    // layer (elevation-only since the TerrainTiles migration).
    auto pack = [](int r, int g, int b) -> float {
        const uint32_t rgb = (static_cast<uint32_t>(r) << 16) |
                             (static_cast<uint32_t>(g) << 8) |
                             static_cast<uint32_t>(b);
        float f;
        std::memcpy(&f, &rgb, sizeof(f));
        return f;
    };
    // Danger ramp stays WARM only (red at floor 0 -> yellow near agl_max):
    // green is reserved for the explicit safe category so "green = safe"
    // never collides with "high-but-still-visible".
    auto rampColor = [&pack](double t) -> float {
        const double h = 60.0 * std::clamp(t, 0.0, 1.0) / 60.0;
        const double s = 0.90, v = 0.95;
        const int i = static_cast<int>(h);
        const double f = h - i;
        const double p = v * (1.0 - s);
        const double q = v * (1.0 - s * f);
        const double u = v * (1.0 - s * (1.0 - f));
        double r = v, g = p, b = p;
        switch (i) {
            case 0: r = v; g = u; b = p; break;
            case 1: r = q; g = v; b = p; break;
            case 2: r = p; g = v; b = u; break;
            default: r = p; g = q; b = v; break;  // 180..185 deg
        }
        return pack(static_cast<int>(255.0 * r + 0.5),
                    static_cast<int>(255.0 * g + 0.5),
                    static_cast<int>(255.0 * b + 0.5));
    };
    const float shadow_color = pack(40, 95, 215);
    const float safe_color = pack(72, 204, 106);

    // Same visibility contour as the marker modes: the floor follows the
    // configured sigmoid threshold, not necessarily the midpoint.
    double vis_offset = 0.0;
    if (settings.threshold > 1e-6 &&
        settings.threshold < 1.0 - 1e-6) {
        vis_offset = settings.softness * std::log(
            settings.threshold / (1.0 - settings.threshold));
    }
    const double rv_ratio = settings.vertical_ratio;

    // [PREVIEW] No DEM yet (before the first plan loads the corridor map):
    // every column falls back to sea level, so the drape would be BURIED
    // inside the rendered terrain mesh — the ideal-field preview was
    // invisible exactly when the user wants to compare it against the
    // terrain-masked result. Float the whole sheet above the tallest
    // rendered terrain instead. Purely a render height; the field itself is
    // the same ideal (unmasked) field it always was.
    const double preview_lift =
        field.terrain.valid ? 0.0 : settings.heatmap_preview_lift;

    size_t painted = 0;
    for (int row = 0; row < rows; ++row) {
        const double wx = x0 + length_x - (row + 0.5) * res;
        for (int col = 0; col < cols; ++col) {
            const double wy = y0 + length_y - (col + 0.5) * res;
            // Water/off-DEM columns sit at sea level 0 (same rule as
            // refreshEffectiveRiskZones / the marker modes).
            double ground = 0.0;
            // Render height uses the MAX elevation within the cell, not the
            // center: on steep ridges the terrain relief inside one coarse
            // heatmap cell exceeds even the 100 m drape offset, so the
            // terrain mesh pierced the drape and z-fought along the
            // intersection line (herringbone bands tracing ridge lines).
            // Semantic quantities (floor AGL, colors) keep the CENTER
            // elevation; only the drawn surface rides the in-cell crest.
            double ground_render = 0.0;
            if (field.terrain.valid) {
                const float h = field.terrain.elevation(wx, wy);
                if (std::isfinite(h) && h > 0.0f)
                    ground = static_cast<double>(h);
                ground_render = ground;
                const double hr = 0.5 * res;
                for (const auto &d :
                     {std::pair<double, double>{-hr, -hr}, {-hr, hr},
                      {hr, -hr}, {hr, hr}}) {
                    const float hc = field.terrain.elevation(
                        wx + d.first, wy + d.second);
                    if (std::isfinite(hc))
                        ground_render = std::max(
                            ground_render, static_cast<double>(hc));
                }
            }
            bool in_footprint = false;
            double floor_agl = kInf;
            for (size_t zi = 0; zi < field.zones.size(); ++zi) {
                const auto &zone = field.zones[zi];
                if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
                const double dx = wx - zone.center.x();
                const double dy = wy - zone.center.y();
                const double rho2 = dx * dx + dy * dy;
                if (rho2 >= zone.reach * zone.reach) continue;
                in_footprint = true;
                const double rv = zone.reach * rv_ratio;
                const double half_z = rv * std::sqrt(std::max(
                    0.0, 1.0 - rho2 / (zone.reach * zone.reach)));
                const double upper = zone.center.z() + half_z;
                if (upper <= ground) continue;  // envelope underground here
                double det = std::max(zone.center.z() - half_z, ground);
                const double horizon = field.shadow_ceiling(
                    zi, Eigen::Vector3d(wx, wy, zone.center.z()));
                if (std::isfinite(horizon))
                    det = std::max(det, horizon + vis_offset);
                if (det < upper)
                    floor_agl = std::min(floor_agl, det - ground);
            }
            if (!in_footprint) continue;  // NaN = transparent cell
            float cell_color;
            if (!std::isfinite(floor_agl)) {
                cell_color = shadow_color;  // never visible here
            } else if (floor_agl >= settings.heatmap_agl_max) {
                // Safe within the band of interest — seen only if you climb
                // above agl_max. Optionally transparent to maximise imagery.
                if (settings.heatmap_safe_transparent) continue;
                cell_color = safe_color;
            } else {
                cell_color = rampColor(floor_agl / settings.heatmap_agl_max);
            }
            const size_t idx =
                static_cast<size_t>(col) * rows + row;
            // Drape offset keeps the heatmap clear of the rendered terrain
            // mesh (which locally overshoots the bilinear DEM on slopes)
            // without reading as a floating slab.
            //
            // [HEATMAP3D] mode "heatmap3d": render the VISIBILITY CEILING as a
            // real 3-D surface instead of a terrain drape — surface altitude
            // = ground + visibility-boundary AGL ("fly BELOW this roof and no
            // zone sees you"). Terrain-occluded (never visible) and
            // safe-in-band
            // cells cap flat at agl_max, so the roof height itself is the
            // information: hugging terrain (red) = visible at any altitude,
            // high roof (green/blue) = generous low-exposure envelope. The
            // trajectory piercing the roof = becoming visible.
            double lift = settings.heatmap_offset;
            if (hm3d) {
                lift = std::isfinite(floor_agl)
                    ? std::min(floor_agl, settings.heatmap_agl_max)
                    : settings.heatmap_agl_max;
                // Red (floor≈0) cells sit ON the terrain: keep a sliver of
                // clearance so the roof does not z-fight the terrain mesh.
                lift = std::max(lift, 0.08);
            }
            elevation.data[idx] =
                static_cast<float>(ground_render + lift + preview_lift);
            color.data[idx] = cell_color;
            ++painted;
        }
    }
    msg.data = {elevation, color};
    if (true) {
        result.diagnostic = formatDiagnostic(
            "[RISK-HEATMAP] %dx%d cells (res=%.3f), %zu painted, "
            "agl_max=%.2f u", rows, cols, res, painted,
            settings.heatmap_agl_max);
    }
    return result;
}

visualization_msgs::msg::MarkerArray makeRiskField(
    const RiskView &field, const RiskSettings &settings, const Stamp &stamp)
{
    require(static_cast<bool>(stamp), "risk field clock is missing");
    validateZones(field);
    if (!field.zones.empty()) {
        validateRiskGeometry(settings);
        validateTerrain(field.terrain);
        require(static_cast<bool>(field.shadow_ceiling) && static_cast<bool>(field.visibility) && static_cast<bool>(field.value), "risk field callbacks are missing");
        require(std::isfinite(settings.step) && std::isfinite(settings.slice_offset) &&
                std::isfinite(settings.agl) && std::isfinite(settings.volume_alpha) &&
                std::isfinite(settings.heatmap_offset), "risk field settings must be finite");
    }

    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.header.frame_id = "map";
    clear.header.stamp = stamp();
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);

    // The draped GridMap channel (self-clearing when mode != "heatmap").
    // The ROS adapter publishes the heatmap before preparing this field.

    const bool volume_mode = settings.mode == "volume";
    const bool heatmap_mode = settings.mode == "heatmap" ||
                              settings.mode == "heatmap3d";
    constexpr double kTwoPi = 6.28318530717958647692;
    for (size_t zi = 0; zi < field.zones.size(); ++zi) {
        const auto &zone = field.zones[zi];
        if (!(zone.reach > 0.0) || !(zone.peak > 0.0)) continue;
        const double rv = zone.reach * settings.vertical_ratio;
        const double auto_step = std::max(
            field.terrain.valid ? field.terrain.resolution : 0.0,
            zone.reach / 70.0);
        const double step = std::max(
            0.05, settings.step > 0.0 ? settings.step
                                             : auto_step);
        const auto header = [&]() {
            std_msgs::msg::Header h;
            h.frame_id = "map";
            h.stamp = stamp();
            return h;
        }();
        auto groundAt = [&field](double x, double y, double *ground) -> bool {
            if (!field.terrain.valid) return false;
            const float h = field.terrain.elevation(x, y);
            if (!std::isfinite(h)) return false;
            *ground = std::max(0.0, static_cast<double>(h));
            return true;
        };
        auto point = [](const Eigen::Vector3d &p) {
            geometry_msgs::msg::Point gp;
            gp.x = p.x(); gp.y = p.y(); gp.z = p.z();
            return gp;
        };
        auto riskColor = [&settings, &zone](double risk, double alpha_scale) {
            std_msgs::msg::ColorRGBA c;
            const double peak = std::max(1e-6, std::min(zone.peak, 1.0));
            const double t = std::clamp(risk / peak, 0.0, 1.0);
            c.r = 1.0f;
            c.g = static_cast<float>(0.38 * (1.0 - t));
            c.b = static_cast<float>(0.04 * (1.0 - t));
            c.a = static_cast<float>(
                settings.volume_alpha * alpha_scale *
                (0.35 + 0.65 * std::sqrt(t)));
            return c;
        };

        if (volume_mode) {
            // The visible lower boundary of the effective 3-D volume:
            // max(ellipsoid floor, terrain, radial-horizon ceiling). Drawing
            // this surface, rather than an opaque top lid, makes terrain
            // shadows legible and avoids the old floating-disc appearance.
            // (Mode "heatmap" carries this same surface as a draped GridMap
            // instead — see publishRiskHeatmap.)
            visualization_msgs::msg::Marker floor;
            floor.header = header;
            floor.ns = "effective_risk_floor";
            floor.id = static_cast<int>(zi);
            floor.type = visualization_msgs::msg::Marker::TRIANGLE_LIST;
            floor.action = visualization_msgs::msg::Marker::ADD;
            floor.pose.orientation.w = 1.0;
            floor.scale.x = floor.scale.y = floor.scale.z = 1.0;
            floor.lifetime = rclcpp::Duration(0, 0);

            // The plotted visibility boundary follows the configured contour,
            // not necessarily the sigmoid midpoint.
            double visibility_z_offset = 0.0;
            if (settings.threshold > 1e-6 &&
                settings.threshold < 1.0 - 1e-6) {
                visibility_z_offset = settings.softness * std::log(
                    settings.threshold /
                    (1.0 - settings.threshold));
            }
            struct SurfaceVertex {
                Eigen::Vector3d p{Eigen::Vector3d::Zero()};
                double risk{0.0};
                bool valid{false};
            };
            auto surfaceAt = [&](double x, double y) {
                SurfaceVertex out;
                const double dx = x - zone.center.x();
                const double dy = y - zone.center.y();
                const double rho2 = dx * dx + dy * dy;
                if (rho2 >= zone.reach * zone.reach) return out;
                const double half_z = rv * std::sqrt(std::max(
                    0.0, 1.0 - rho2 / (zone.reach * zone.reach)));
                double lower = zone.center.z() - half_z;
                const double upper = zone.center.z() + half_z;
                double ground = 0.0;
                if (groundAt(x, y, &ground))
                    lower = std::max(lower, ground + 0.02);
                const double horizon = field.shadow_ceiling(
                    zi, Eigen::Vector3d(x, y, zone.center.z()));
                if (std::isfinite(horizon))
                    lower = std::max(lower,
                                     horizon + visibility_z_offset);
                if (!(lower < upper)) return out;
                out.p = Eigen::Vector3d(x, y, lower + 0.01);
                out.risk = field.value(zi, out.p);
                out.valid = true;
                return out;
            };
            auto appendTriangle = [&](const SurfaceVertex &a,
                                      const SurfaceVertex &b,
                                      const SurfaceVertex &c) {
                if (!a.valid || !b.valid || !c.valid) return;
                // Do not bridge a cliff-sized discontinuity in the horizon
                // table; leaving a narrow gap is visually more honest.
                const double zmin = std::min({a.p.z(), b.p.z(), c.p.z()});
                const double zmax = std::max({a.p.z(), b.p.z(), c.p.z()});
                if (zmax - zmin > std::max(3.0 * step, 0.25 * rv)) return;
                for (const auto *v : {&a, &b, &c}) {
                    floor.points.push_back(point(v->p));
                    floor.colors.push_back(riskColor(v->risk, 1.0));
                }
            };
            const int half_cells = std::min(
                180, checkedInt(std::ceil(zone.reach / step)));
            const double mesh_step = zone.reach /
                static_cast<double>(std::max(1, half_cells));
            for (int iy = -half_cells; iy < half_cells; ++iy) {
                const double y0 = zone.center.y() + iy * mesh_step;
                const double y1 = y0 + mesh_step;
                for (int ix = -half_cells; ix < half_cells; ++ix) {
                    const double x0 = zone.center.x() + ix * mesh_step;
                    const double x1 = x0 + mesh_step;
                    const SurfaceVertex v00 = surfaceAt(x0, y0);
                    const SurfaceVertex v10 = surfaceAt(x1, y0);
                    const SurfaceVertex v01 = surfaceAt(x0, y1);
                    const SurfaceVertex v11 = surfaceAt(x1, y1);
                    appendTriangle(v00, v10, v11);
                    appendTriangle(v00, v11, v01);
                }
            }
            if (!floor.points.empty()) arr.markers.push_back(std::move(floor));
        }

        if (volume_mode || heatmap_mode) {
            // Sparse altitude contours plus meridians communicate the full
            // ellipsoid without filling it with a visually dominant lid.
            // Heatmap mode keeps only the visibility-clipped equator ring:
            // the draped map already carries the field, so the ring just
            // marks the risk boundary.
            const int n_contours = heatmap_mode ? 1 : settings.contours;
            const int n_meridians = heatmap_mode ? 0 : 12;
            visualization_msgs::msg::Marker wire;
            wire.header = header;
            wire.ns = "effective_risk_volume";
            wire.id = static_cast<int>(zi);
            wire.type = visualization_msgs::msg::Marker::LINE_LIST;
            wire.action = visualization_msgs::msg::Marker::ADD;
            wire.pose.orientation.w = 1.0;
            // Readability: the rim used to be a semi-transparent red line on
            // the warm draped heatmap — red on red. Opaque, wider, and
            // backed by a darker halo underlay (published below) so the
            // boundary reads on any background.
            //
            // In heatmap mode the ring is also DRAPED onto the terrain (see
            // drapeZ): drawn at the ellipsoid equator's constant altitude it
            // floated over valleys and sank into ridges, reading as a chain
            // hanging in mid-air rather than a boundary. Membership and
            // visibility are still evaluated on the TRUE ellipsoid geometry;
            // only the drawn height follows the ground.
            wire.scale.x = std::max(0.14, 0.22 * step);
            wire.color.r = 1.0f;
            wire.color.g = 0.18f;
            wire.color.b = 0.02f;
            wire.color.a = 1.0f;
            wire.lifetime = rclcpp::Duration(0, 0);
            auto shellPointVisible = [&](const Eigen::Vector3d &p) {
                double ground = 0.0;
                if (groundAt(p.x(), p.y(), &ground) &&
                    p.z() <= ground + 0.02) return false;
                return field.visibility(zi, p) >=
                       settings.threshold;
            };
            // Drawn height: terrain-following in heatmap mode (rides just
            // above the drape), true geometry otherwise (volume/wire modes
            // are showing the 3-D shell, where floating IS the point).
            auto drapeZ = [&](const Eigen::Vector3d &p) {
                if (!heatmap_mode) return p;
                double ground = 0.0;
                groundAt(p.x(), p.y(), &ground);
                Eigen::Vector3d q = p;
                q.z() = ground + settings.heatmap_offset + 0.05;
                return q;
            };
            auto appendSegment = [&](const Eigen::Vector3d &a,
                                     const Eigen::Vector3d &b) {
                if (!shellPointVisible(a) || !shellPointVisible(b)) return;
                wire.points.push_back(point(drapeZ(a)));
                wire.points.push_back(point(drapeZ(b)));
            };
            const int ring_samples = 192;
            for (int li = 0; li < n_contours; ++li) {
                const double f = n_contours == 1 ? 0.0 :
                    -0.70 + 1.40 * li /
                    static_cast<double>(n_contours - 1);
                const double z = zone.center.z() + f * rv;
                const double radius = zone.reach *
                    std::sqrt(std::max(0.0, 1.0 - f * f));
                for (int ai = 0; ai < ring_samples; ++ai) {
                    const double a0 = kTwoPi * ai / ring_samples;
                    const double a1 = kTwoPi * (ai + 1) / ring_samples;
                    appendSegment(
                        Eigen::Vector3d(zone.center.x() + radius * std::cos(a0),
                                        zone.center.y() + radius * std::sin(a0), z),
                        Eigen::Vector3d(zone.center.x() + radius * std::cos(a1),
                                        zone.center.y() + radius * std::sin(a1), z));
                }
            }
            constexpr int kMeridianSamples = 48;
            for (int mi = 0; mi < n_meridians; ++mi) {
                const double az = kTwoPi * mi / n_meridians;
                for (int bi = 0; bi < kMeridianSamples; ++bi) {
                    const double b0 = -0.5 * M_PI +
                        M_PI * bi / kMeridianSamples;
                    const double b1 = -0.5 * M_PI +
                        M_PI * (bi + 1) / kMeridianSamples;
                    appendSegment(
                        Eigen::Vector3d(
                            zone.center.x() + zone.reach * std::cos(b0) * std::cos(az),
                            zone.center.y() + zone.reach * std::cos(b0) * std::sin(az),
                            zone.center.z() + rv * std::sin(b0)),
                        Eigen::Vector3d(
                            zone.center.x() + zone.reach * std::cos(b1) * std::cos(az),
                            zone.center.y() + zone.reach * std::cos(b1) * std::sin(az),
                            zone.center.z() + rv * std::sin(b1)));
                }
            }
            if (!wire.points.empty()) {
                // Cartographic halo: a wider near-black underlay one step
                // closer to the drape; the bright rim renders on top of it.
                visualization_msgs::msg::Marker halo = wire;
                halo.ns = "effective_risk_volume_halo";
                halo.scale.x = 2.0 * wire.scale.x;
                halo.pose.position.z = -0.01;
                halo.color.r = 0.08f;
                halo.color.g = 0.0f;
                halo.color.b = 0.0f;
                halo.color.a = 0.85f;
                arr.markers.push_back(std::move(halo));
                arr.markers.push_back(std::move(wire));
            }
        }

        if (!volume_mode && !heatmap_mode) {
            // Explicit diagnostic cross-section. POINTS avoid the thick,
            // staircase-like CUBE_LIST slab that previously looked physical.
            visualization_msgs::msg::Marker slice;
            slice.header = header;
            slice.ns = settings.mode == "fixed_agl"
                           ? "effective_risk_fixed_agl"
                           : "effective_risk_fixed_msl";
            slice.id = static_cast<int>(zi);
            slice.type = visualization_msgs::msg::Marker::POINTS;
            slice.action = visualization_msgs::msg::Marker::ADD;
            slice.pose.orientation.w = 1.0;
            slice.scale.x = slice.scale.y = std::max(0.08, 0.70 * step);
            slice.lifetime = rclcpp::Duration(0, 0);
            const double slice_z =
                zone.center.z() + settings.slice_offset;
            const int half_cells = checkedInt(
                std::ceil(zone.reach / step));
            for (int iy = -half_cells; iy <= half_cells; ++iy) {
                const double y = zone.center.y() + iy * step;
                for (int ix = -half_cells; ix <= half_cells; ++ix) {
                    const double x = zone.center.x() + ix * step;
                    double z = slice_z;
                    if (settings.mode == "fixed_agl") {
                        double ground = 0.0;
                        if (!groundAt(x, y, &ground)) continue;
                        z = ground + settings.agl;
                    }
                    const Eigen::Vector3d p(x, y, z);
                    if (field.visibility(zi, p) <
                        settings.threshold) continue;
                    const double risk = field.value(zi, p);
                    if (risk < 1e-3) continue;
                    slice.points.push_back(point(
                        Eigen::Vector3d(x, y, z + 0.02)));
                    slice.colors.push_back(riskColor(risk, 2.0));
                }
            }
            if (!slice.points.empty()) arr.markers.push_back(std::move(slice));
        }

        visualization_msgs::msg::Marker source;
        source.header = header;
        source.ns = "risk_source";
        source.id = static_cast<int>(zi);
        source.type = visualization_msgs::msg::Marker::SPHERE;
        source.action = visualization_msgs::msg::Marker::ADD;
        source.pose.position.x = zone.center.x();
        source.pose.position.y = zone.center.y();
        source.pose.position.z = zone.center.z();
        source.pose.orientation.w = 1.0;
        source.scale.x = std::max(0.5, 0.008 * zone.reach);
        source.scale.y = source.scale.x;
        source.scale.z = source.scale.x;
        source.color.r = 0.8f;
        source.color.g = 0.0f;
        source.color.b = 0.0f;
        source.color.a = 1.0f;
        source.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(std::move(source));

        // A thin vertical reference visually anchors a ground risk source.
        double source_ground = 0.0;
        if (groundAt(zone.center.x(), zone.center.y(), &source_ground) &&
            zone.center.z() > source_ground + 0.02) {
            visualization_msgs::msg::Marker mast;
            mast.header = header;
            mast.ns = "risk_source_mast";
            mast.id = static_cast<int>(zi);
            mast.type = visualization_msgs::msg::Marker::LINE_LIST;
            mast.action = visualization_msgs::msg::Marker::ADD;
            mast.pose.orientation.w = 1.0;
            mast.scale.x = std::max(0.06, 0.08 * step);
            mast.color.r = 0.8f;
            mast.color.g = 0.05f;
            mast.color.b = 0.02f;
            mast.color.a = 0.9f;
            mast.points.push_back(point(Eigen::Vector3d(
                zone.center.x(), zone.center.y(), source_ground)));
            mast.points.push_back(point(zone.center));
            mast.lifetime = rclcpp::Duration(0, 0);
            arr.markers.push_back(std::move(mast));
        }

        visualization_msgs::msg::Marker label;
        label.header = header;
        label.ns = "risk_label";
        label.id = static_cast<int>(zi);
        label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
        label.action = visualization_msgs::msg::Marker::ADD;
        label.pose.position.x = zone.center.x();
        label.pose.position.y = zone.center.y();
        label.pose.position.z = zone.center.z() + rv +
                                std::max(1.0, 0.03 * zone.reach);
        label.pose.orientation.w = 1.0;
        label.scale.z = std::max(1.0, 0.02 * zone.reach);
        label.color.r = 1.0f;
        label.color.g = 0.2f;
        label.color.b = 0.2f;
        label.color.a = 1.0f;
        char label_text[96];
        if (volume_mode || heatmap_mode) {
            std::snprintf(label_text, sizeof(label_text),
                          "terrain-occluded risk zone #%zu  Rh=%.0fm Rv=%.0fm",
                          zi, 100.0 * zone.reach, 100.0 * rv);
        } else if (settings.mode == "fixed_agl") {
            std::snprintf(label_text, sizeof(label_text),
                          "risk #%zu @ terrain + %.0f m AGL",
                          zi, 100.0 * settings.agl);
        } else {
            std::snprintf(label_text, sizeof(label_text),
                          "risk #%zu @ z=%.2f MSL", zi,
                          zone.center.z() + settings.slice_offset);
        }
        label.text = label_text;
        label.lifetime = rclcpp::Duration(0, 0);
        arr.markers.push_back(std::move(label));
    }

    // Single atomic repaint. With no zones this is the lone DELETEALL, which
    // is exactly the clear that EmergencyStop / a world change / an empty
    // runtime zone set need.
    return arr;
}

visualization_msgs::msg::MarkerArray makeDynamicObstacles(
    const std::vector<Obstacle> &obstacles, double sphere_mesh_height,
    const Stamp &stamp)
{
    require(static_cast<bool>(stamp), "obstacle clock is missing");
    for (const auto &obstacle : obstacles) {
        require(obstacle.center.allFinite() && obstacle.size.allFinite() &&
                (obstacle.size.array() > 0.0).all() && std::isfinite(obstacle.yaw) &&
                obstacle.mesh_native_size.allFinite() && (obstacle.mesh_native_size.array() > 0.0).all() &&
                std::isfinite(obstacle.mesh_scale), "visual obstacle dimensions/orientation are invalid");
        if (!obstacle.is_box) require(std::isfinite(sphere_mesh_height),
                                     "sphere mesh height must be finite");
    }
    visualization_msgs::msg::MarkerArray arr;

    // Single DELETEALL marker first so removed patches disappear in RViz.
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.header.frame_id = "map";
    clear_marker.header.stamp = stamp();
    clear_marker.ns = "dynamic_obstacles";
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear_marker);

    for (const auto &obstacle : obstacles) {
        // Per-obstacle visual model from the catalog; collision (SDF) is separate.
        // Mesh descriptors are projected by the caller, independently of collision.
        visualization_msgs::msg::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = stamp();
        m.ns = "dynamic_obstacles";
        m.id = obstacle.id;
        // color all-zero => RViz uses the mesh's own embedded materials/textures.
        m.type = visualization_msgs::msg::Marker::MESH_RESOURCE;
        m.mesh_resource = obstacle.mesh_resource;
        m.mesh_use_embedded_materials = true;
        m.action = visualization_msgs::msg::Marker::ADD;
        m.color.r = m.color.g = m.color.b = m.color.a = 0.0f;  // use mesh textures

        const Eigen::Vector3d& c = obstacle.center;
        double foot_x, foot_y, height, base_z;
        if (obstacle.is_box) {
            // Box: mesh fills the collision box exactly (collision == visual bbox).
            foot_x = obstacle.size.x();
            foot_y = obstacle.size.y();
            height = obstacle.size.z();
            base_z = c.z() - 0.5 * height;
        } else {
            // Sphere: fixed-height building from the ball's bottom.
            const double radius = 0.5 * obstacle.size.x();  // stored as (2r,2r,2r)
            foot_x = foot_y = 2.0 * radius;
            height = sphere_mesh_height;
            base_z = c.z() - radius;
        }

        // Visual-only magnification: grows the mesh about the BASE (xy about
        // the center, z upward from the bottom face) so magnified obstacles
        // keep resting on the surface — center-anchored scaling pushed half
        // of every magnified hull underground/underwater. Per-model via
        // obstacle_viz_scale_<model>, global via obstacle_viz_scale.
        // Collision, companion geometry marker and panel overlay untouched.
        const double vs = std::max(1.0, obstacle.mesh_scale);
        m.pose.position.x = c.x();
        m.pose.position.y = c.y();
        m.pose.position.z = base_z;
        // Per-spawn yaw about Z, shared with the SDF collision primitive
        // (PrimitiveSpec.yaw) — what you see is what the planner avoids.
        const double yaw = obstacle.yaw;
        m.pose.orientation.z = std::sin(0.5 * yaw);
        m.pose.orientation.w = std::cos(0.5 * yaw);
        m.scale.x = vs * foot_x / obstacle.mesh_native_size.x();
        m.scale.y = vs * foot_y / obstacle.mesh_native_size.y();
        m.scale.z = vs * height / obstacle.mesh_native_size.z();
        arr.markers.push_back(m);

        // Companion GEOMETRY marker: the exact collision primitive (center /
        // size / yaw), fully transparent so RViz shows only the mesh above.
        // Panels cannot recover the collision box from a MESH_RESOURCE marker
        // (its scale is relative to unknown native mesh dims), and without
        // this the altitude profile cannot explain box-avoidance climbs —
        // "why is it climbing over open water" turned out to be invisible
        // ships more than once.
        visualization_msgs::msg::Marker g;
        g.header = m.header;
        g.ns = "dynamic_obstacles_geom";
        g.id = obstacle.id;
        g.type = visualization_msgs::msg::Marker::CUBE;
        g.action = visualization_msgs::msg::Marker::ADD;
        g.pose.position.x = c.x();
        g.pose.position.y = c.y();
        g.pose.position.z = c.z();
        g.pose.orientation = m.pose.orientation;
        g.scale.x = obstacle.is_box ? obstacle.size.x() : foot_x;
        g.scale.y = obstacle.is_box ? obstacle.size.y() : foot_y;
        g.scale.z = obstacle.is_box ? obstacle.size.z()
                                         : obstacle.size.x();
        g.color.a = 0.0f;  // invisible in RViz; geometry carrier only
        arr.markers.push_back(g);
    }
    return arr;
}

visualization_msgs::msg::Marker makeTrajectoryClear()
{
    visualization_msgs::msg::Marker del;
    del.header.frame_id = "world";
    del.action = visualization_msgs::msg::Marker::DELETEALL;
    return del;
}

}  // namespace path_manager::visualization
