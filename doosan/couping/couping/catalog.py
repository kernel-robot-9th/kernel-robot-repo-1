"""주문 가능한 품목 목록 (번호 = YOLO 클래스 순서 + 1)."""

ITEMS = ('tomato', 'carrot', 'salad', 'bath_tissue', 'tide', 'paper_towel')
KOREAN = {'tomato': '토마토', 'carrot': '당근', 'salad': '샐러드',
          'bath_tissue': '휴지', 'tide': '세제', 'paper_towel': '키친타월'}


def label(item):
    return f'{KOREAN.get(item, item)}({item})'


def parse(text):
    """'1, 3' / '1 3' / '13' / 'tomato,tide' → ['tomato', 'salad']. 잘못된 입력은 ValueError."""
    items = []
    for tok in text.replace(',', ' ').split():
        if tok in ITEMS:
            items.append(tok)
        elif tok.isdigit():
            for ch in tok:  # '13' → 1, 3 (품목이 9개 이하라 한 자리씩)
                n = int(ch)
                if not 1 <= n <= len(ITEMS):
                    raise ValueError(f'{n}번 품목은 없습니다 (1~{len(ITEMS)})')
                items.append(ITEMS[n - 1])
        else:
            raise ValueError(f"'{tok}' 은(는) 품목 번호가 아닙니다")
    if not items:
        raise ValueError('품목을 하나 이상 입력하세요')
    return items
