from det_ros.labels import annotated_topic_for


def test_annotated_topic_next_to_source_image():
    assert annotated_topic_for('/left_oa_camera/image_raw') == '/left_oa_camera/image_annotated'
    assert annotated_topic_for('right_oa_camera/image_raw') == '/right_oa_camera/image_annotated'
    assert annotated_topic_for('/robot/front/image_rect') == '/robot/front/image_annotated'
    assert annotated_topic_for('/image') == '/image_annotated'
