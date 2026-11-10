from uav_info.scan_layer_relay import planner_scan_topics


def test_scan_relay_topics_cover_each_planner_band_for_one_uav():
    assert planner_scan_topics('uav_1') == {
        3: '/planner_3m/uav_1/scan',
        5: '/planner_5m/uav_1/scan',
        7: '/planner_7m/uav_1/scan',
        10: '/planner_10m/uav_1/scan',
    }


def test_scan_relay_topics_are_namespaced_independently_for_each_uav():
    first = planner_scan_topics('uav_1')
    second = planner_scan_topics('uav_2')

    assert set(first) == set(second) == {3, 5, 7, 10}
    assert all('/uav_1/' in topic for topic in first.values())
    assert all('/uav_2/' in topic for topic in second.values())
